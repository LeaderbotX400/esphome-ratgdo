#pragma once

#ifdef PROTOCOL_SECPLUSV1

#include <queue>

#include "SoftwareSerial.h" // Using espsoftwareserial https://github.com/plerup/espsoftwareserial
#include "esphome/core/optional.h"

#include "callbacks.h"
#include "observable.h"
#include "protocol.h"
#include "ratgdo_state.h"

namespace esphome {

class Scheduler;
class InternalGPIOPin;

namespace ratgdo {
    namespace secplus1 {

        using namespace esphome::ratgdo::protocol;

        // ─── Serial Configuration ──────────────────────────────────────────
        static const uint32_t BAUD_RATE = 1200;

        // ─── Timing Constants (milliseconds) ──────────────────────────────
        static const uint32_t EMULATION_CYCLE_MS = 250; // wall panel emulation interval
        static const uint32_t MIN_TX_INTERVAL_MS = 200; // minimum gap between transmissions
        static const uint32_t POST_RX_QUIET_MS = 50; // quiet period after receiving before transmitting
        static const uint32_t RX_TIMEOUT_MS = 100; // discard partial packet if no new byte arrives
        static const uint32_t SYNC_TIMEOUT_MS = 45000; // maximum time to wait for initial sync
        static const uint32_t DOOR_RELEASE_DELAY_MS = 500; // delay before sending door toggle release
        static const uint32_t LIGHT_RELEASE_DELAY_MS = 500; // delay before sending light toggle release
        static const uint32_t LOCK_RELEASE_DELAY_MS = 3500; // delay before sending lock toggle release
        static const uint32_t STATUS_QUERY_INTERVAL_MS = 10000; // interval between status queries on alternate panels

        // ─── Packet Dimensions ─────────────────────────────────────────────
        static const uint8_t PACKET_LENGTH = 2;

        // ─── Command Byte Range ────────────────────────────────────────────
        static const uint8_t CMD_BYTE_MIN = 0x30;
        static const uint8_t CMD_BYTE_MAX = 0x3A;

        // ─── Door Status Bit Masks ─────────────────────────────────────────
        static const uint8_t DOOR_STATUS_MASK = 0x07; // bits 0-2 encode door state
        static const uint8_t DOOR_STATUS_STOPPED_A = 0x00;
        static const uint8_t DOOR_STATUS_OPENING = 0x01;
        static const uint8_t DOOR_STATUS_OPEN = 0x02;
        static const uint8_t DOOR_STATUS_CLOSING = 0x04;
        static const uint8_t DOOR_STATUS_CLOSED = 0x05;
        static const uint8_t DOOR_STATUS_STOPPED_B = 0x06;

        // ─── Light / Lock Status Bit Positions ─────────────────────────────
        static const uint8_t LIGHT_STATUS_BIT = 2; // bit 2 of QUERY_OTHER_STATUS response
        static const uint8_t LOCK_STATUS_BIT = 3; // bit 3 (inverted) of QUERY_OTHER_STATUS response

        // ─── Command Types ─────────────────────────────────────────────────
        ENUM(CommandType, uint8_t,
            (TOGGLE_DOOR_PRESS, 0x30),
            (TOGGLE_DOOR_RELEASE, 0x31),
            (TOGGLE_LIGHT_PRESS, 0x32),
            (TOGGLE_LIGHT_RELEASE, 0x33),
            (TOGGLE_LOCK_PRESS, 0x34),
            (TOGGLE_LOCK_RELEASE, 0x35),
            (QUERY_DOOR_STATUS_ALT, 0x37),
            (QUERY_DOOR_STATUS, 0x38),
            (OBSTRUCTION, 0x39),
            (QUERY_OTHER_STATUS, 0x3A),
            (UNKNOWN, 0xFF), )

        // ─── Wall Panel Emulation Byte Sequence ────────────────────────────
        //
        // The wall panel continuously sends this byte sequence, one byte per
        // EMULATION_CYCLE_MS interval. The first EMULATION_INIT_LENGTH bytes
        // are an initialization preamble. After that, the sequence loops over
        // the last EMULATION_LOOP_LENGTH bytes, which contain the status
        // query commands (door, obstruction, and light/lock status).
        //
        static const uint8_t EMULATION_SEQUENCE[] PROGMEM = {
            0x35,
            0x35,
            0x35,
            0x35, // lock release (preamble padding)
            0x33,
            0x33, // light release
            0x53,
            0x53, // unknown handshake bytes
            0x38, // query door status
            0x3A,
            0x3A,
            0x3A, // query other status (×3)
            0x39, // obstruction check
            0x38, // query door status
            0x3A, // query other status
            // ── loop section (indices 15-17) ──
            0x38, // query door status
            0x3A, // query other status
            0x39, // obstruction check
        };
        static const size_t EMULATION_INIT_LENGTH = 15; // preamble bytes before the main loop
        static const size_t EMULATION_TOTAL_LENGTH = sizeof(EMULATION_SEQUENCE);
        static_assert(sizeof(EMULATION_SEQUENCE) > EMULATION_INIT_LENGTH,
            "EMULATION_SEQUENCE must contain more bytes than the initialization preamble");
        static const size_t EMULATION_LOOP_LENGTH = EMULATION_TOTAL_LENGTH - EMULATION_INIT_LENGTH;

        // ─── Received Command ──────────────────────────────────────────────
        struct RxCommand {
            CommandType type;
            uint8_t data;

            RxCommand()
                : type(CommandType::UNKNOWN)
                , data(0)
            {
            }
            RxCommand(CommandType type_, uint8_t data_ = 0)
                : type(type_)
                , data(data_)
            {
            }
        };

        // ─── Scheduled Transmission ────────────────────────────────────────
        struct ScheduledTx {
            CommandType command;
            uint32_t send_at; // millis() timestamp when this command becomes eligible

            bool operator>(const ScheduledTx& other) const { return send_at > other.send_at; }
        };

        struct ScheduledTxGreater {
            bool operator()(const ScheduledTx& a, const ScheduledTx& b) const { return a > b; }
        };

        // ─── Unconfirmed State ─────────────────────────────────────────────
        //
        // On standard wall panels (non-alternate), status values must be seen
        // twice consecutively before being accepted. This struct tracks the
        // most recently observed but not-yet-confirmed value for each state.
        //
        template <typename T>
        struct Unconfirmed {
            T value;

            Unconfirmed(T initial)
                : value(initial)
            {
            }

            // Returns true if the incoming value matches the previously seen
            // unconfirmed value (i.e., it is now confirmed). Always updates
            // the stored value to the latest observation.
            bool confirm(T incoming)
            {
                bool confirmed = (incoming == value);
                value = incoming;
                return confirmed;
            }
        };

        // ─── Packet Receive State ──────────────────────────────────────────
        //
        // Tracks the state of an in-progress packet reception, replacing the
        // static local variables that were previously used in read_command().
        //
        struct RxState {
            uint8_t buffer[PACKET_LENGTH] { };
            uint8_t count { 0 };
            bool reading { false };

            void reset()
            {
                count = 0;
                reading = false;
            }
        };

        // ─── Protocol Implementation ───────────────────────────────────────

        class Secplus1 : public Protocol {
        public:
            void setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin);
            void loop();
            void dump_config();

            void sync();

            void light_action(LightAction action);
            void lock_action(LockAction action);
            void door_action(DoorAction action);

            Result call(Args args);

            const Traits& traits() const { return this->traits_; }

            // Methods not applicable to Security+ 1.0
            void set_open_limit(bool) { }
            void set_close_limit(bool) { }
            void set_discrete_open_pin(InternalGPIOPin*) { }
            void set_discrete_close_pin(InternalGPIOPin*) { }

        protected:
            // ── Wall Panel Emulation ───────────────────────────────────────
            void start_emulation();
            void emulation_step(size_t index);
            uint8_t read_emulation_byte(size_t index) const;

            // ── Receive Path ───────────────────────────────────────────────
            optional<RxCommand> read_command();
            optional<RxCommand> try_read_first_byte();
            optional<RxCommand> try_read_remaining_bytes();
            optional<RxCommand> decode_packet() const;
            void handle_command(const RxCommand& cmd);

            // ── Transmit Path ──────────────────────────────────────────────
            void schedule_tx(CommandType cmd, uint32_t send_at = 0);
            void schedule_press_release_pair(CommandType press_cmd);
            optional<CommandType> peek_pending_tx() const;
            optional<CommandType> pop_pending_tx();
            bool transmit_if_pending();
            void send_byte(uint8_t value);

            // ── Status Handling ────────────────────────────────────────────
            void handle_door_status(const RxCommand& cmd);
            void handle_alternate_query(const RxCommand& cmd);
            void handle_other_status(const RxCommand& cmd);
            void handle_obstruction(const RxCommand& cmd);
            DoorState decode_door_state(uint8_t status_bits) const;

            // ── Action Helpers ─────────────────────────────────────────────
            void send_door_toggle();
            void send_light_toggle();
            void send_lock_toggle();

            // ── Member Variables ───────────────────────────────────────────

            // Pointers (4-byte aligned)
            InternalGPIOPin* tx_pin_ { nullptr };
            InternalGPIOPin* rx_pin_ { nullptr };
            RATGDOComponent* ratgdo_ { nullptr };
            Scheduler* scheduler_ { nullptr };

            // Timestamps (4-byte)
            uint32_t last_rx_time_ { 0 };
            uint32_t last_tx_time_ { 0 };
            uint32_t last_status_query_time_ { 0 };

            // Larger structures
            std::priority_queue<ScheduledTx, std::vector<ScheduledTx>, ScheduledTxGreater> tx_queue_;
            SoftwareSerial sw_serial_;
            OnceCallbacks<void(DoorState)> on_door_state_;
            Traits traits_;
            RxState rx_state_;

            // Unconfirmed state tracking (for standard panels)
            Unconfirmed<DoorState> unconfirmed_door_ { DoorState::UNKNOWN };
            Unconfirmed<LightState> unconfirmed_light_ { LightState::UNKNOWN };
            Unconfirmed<LockState> unconfirmed_lock_ { LockState::UNKNOWN };

            // Confirmed state cache
            DoorState door_state_ { DoorState::UNKNOWN };
            LightState light_state_ { LightState::UNKNOWN };
            LockState lock_state_ { LockState::UNKNOWN };

            // Flags (packed into a single byte)
            struct {
                uint8_t emulation_active : 1;
                uint8_t door_moving : 1;
                uint8_t uses_alternate_query : 1; // true if GDO uses 0x37-style status queries
                uint8_t reserved : 5;
            } flags_ { };
        };

    } // namespace secplus1
} // namespace ratgdo
} // namespace esphome

#endif // PROTOCOL_SECPLUSV1
