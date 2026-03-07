
#ifdef PROTOCOL_SECPLUSV1

#include "secplus1.h"
#include "ratgdo.h"

#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/scheduler.h"

namespace esphome {
namespace ratgdo {
    namespace secplus1 {

        static const char* const TAG = "ratgdo_secplus1";

        // ════════════════════════════════════════════════════════════════════
        //  Lifecycle
        // ════════════════════════════════════════════════════════════════════

        void Secplus1::setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin)
        {
            this->ratgdo_ = ratgdo;
            this->scheduler_ = scheduler;
            this->tx_pin_ = tx_pin;
            this->rx_pin_ = rx_pin;

            this->sw_serial_.begin(BAUD_RATE, SWSERIAL_8E1, rx_pin->get_pin(), tx_pin->get_pin(), true);

            this->traits_.set_features(HAS_DOOR_STATUS | HAS_LIGHT_TOGGLE | HAS_LOCK_TOGGLE);
        }

        void Secplus1::loop()
        {
            auto cmd = this->read_command();
            if (cmd) {
                this->handle_command(cmd.value());
            }

            bool enough_time_since_tx = (millis() - this->last_tx_time_) > MIN_TX_INTERVAL_MS;
            bool enough_time_since_rx = (millis() - this->last_rx_time_) > POST_RX_QUIET_MS;
            auto pending = this->peek_pending_tx();
            bool lock_blocked_by_alt_panel = this->flags_.uses_alternate_query
                && pending
                && pending.value() == CommandType::TOGGLE_LOCK_PRESS;

            if (enough_time_since_tx
                && enough_time_since_rx
                && pending
                && !lock_blocked_by_alt_panel
                && this->flags_.emulation_active) {
                this->transmit_if_pending();
            }
        }

        void Secplus1::dump_config()
        {
            ESP_LOGCONFIG(TAG, "  Protocol: SEC+ v1");
        }

        // ════════════════════════════════════════════════════════════════════
        //  Sync
        // ════════════════════════════════════════════════════════════════════

        void Secplus1::sync()
        {
            this->door_state_ = DoorState::UNKNOWN;
            this->light_state_ = LightState::UNKNOWN;

            this->start_emulation();

            this->scheduler_->set_timeout(this->ratgdo_, "", SYNC_TIMEOUT_MS, [this] {
                if (this->door_state_ == DoorState::UNKNOWN) {
                    ESP_LOGW(TAG, "Triggering sync failed actions.");
                    this->ratgdo_->sync_failed = true;
                }
            });
        }

        // ════════════════════════════════════════════════════════════════════
        //  Wall Panel Emulation
        // ════════════════════════════════════════════════════════════════════
        //
        // A Security+ 1.0 wall panel continuously broadcasts a repeating byte
        // sequence to the GDO. We emulate this behaviour so the GDO stays
        // responsive. The sequence has an initialization preamble followed by
        // a looping status-query section. During the loop, pending user
        // commands may be substituted in place of the next emulation byte.
        //

        void Secplus1::start_emulation()
        {
            this->flags_.emulation_active = true;
            this->scheduler_->cancel_timeout(this->ratgdo_, "wall_panel_emulation");
            this->emulation_step(0);
        }

        void Secplus1::emulation_step(size_t index)
        {
            if (!this->flags_.emulation_active) {
                return;
            }

            // During the loop phase, attempt to send a pending user command
            // instead of the next emulation byte.
            bool in_loop_phase = index >= EMULATION_INIT_LENGTH;
            if (!in_loop_phase || !this->transmit_if_pending()) {
                this->send_byte(this->read_emulation_byte(index));
            }

            // Advance and wrap around into the loop section
            index++;
            if (index >= EMULATION_TOTAL_LENGTH) {
                index = EMULATION_INIT_LENGTH;
            }

            this->scheduler_->set_timeout(this->ratgdo_, "wall_panel_emulation", EMULATION_CYCLE_MS, [this, index] {
                this->emulation_step(index);
            });
        }

        uint8_t Secplus1::read_emulation_byte(size_t index) const
        {
#ifdef USE_ESP8266
            return progmem_read_byte(&EMULATION_SEQUENCE[index]);
#else
            return EMULATION_SEQUENCE[index];
#endif
        }

        // ════════════════════════════════════════════════════════════════════
        //  Receive Path
        // ════════════════════════════════════════════════════════════════════

        optional<RxCommand> Secplus1::read_command()
        {
            if (!this->rx_state_.reading) {
                return this->try_read_first_byte();
            }
            return this->try_read_remaining_bytes();
        }

        optional<RxCommand> Secplus1::try_read_first_byte()
        {
            while (this->sw_serial_.available()) {
                uint8_t byte = this->sw_serial_.read();
                this->last_rx_time_ = millis();

                // Only accept bytes in the valid command range
                if (byte < CMD_BYTE_MIN || byte > CMD_BYTE_MAX) {
                    ESP_LOG2(TAG, "[%d] Ignoring byte [%02X], baud: %d", millis(), byte, this->sw_serial_.baudRate());
                    continue;
                }

                ESP_LOG2(TAG, "[%d] Received byte: [%02X]", millis(), byte);

                // Single-byte commands: 0x30-0x35 (press/release) and 0x37 (alt query)
                // These have no response byte.
                if (byte == static_cast<uint8_t>(CommandType::QUERY_DOOR_STATUS_ALT)
                    || (byte >= static_cast<uint8_t>(CommandType::TOGGLE_DOOR_PRESS)
                        && byte <= static_cast<uint8_t>(CommandType::TOGGLE_LOCK_RELEASE))) {

                    ESP_LOG2(TAG, "[%d] Received single-byte command: [%02X]", millis(), byte);
                    this->rx_state_.buffer[0] = byte;
                    this->rx_state_.buffer[1] = 0;
                    return this->decode_packet();
                }

                // Two-byte command: store the first byte and wait for response
                this->rx_state_.buffer[0] = byte;
                this->rx_state_.count = 1;
                this->rx_state_.reading = true;
                break;
            }
            return { };
        }

        optional<RxCommand> Secplus1::try_read_remaining_bytes()
        {
            while (this->sw_serial_.available()) {
                uint8_t byte = this->sw_serial_.read();
                this->last_rx_time_ = millis();
                this->rx_state_.buffer[this->rx_state_.count++] = byte;
                ESP_LOG2(TAG, "[%d] Received byte: [%02X]", millis(), byte);

                if (this->rx_state_.count >= PACKET_LENGTH) {
                    ESP_LOG2(TAG, "[%d] Received packet: [%02X %02X]",
                        millis(), this->rx_state_.buffer[0], this->rx_state_.buffer[1]);
                    auto result = this->decode_packet();
                    this->rx_state_.reset();
                    return result;
                }
            }

            // Discard stale partial packet (a full packet arrives in ~20 ms)
            if (millis() - this->last_rx_time_ > RX_TIMEOUT_MS) {
                ESP_LOGW(TAG, "[%d] Discarding incomplete packet: [%02X ...]",
                    millis(), this->rx_state_.buffer[0]);
                this->rx_state_.reset();
            }

            return { };
        }

        optional<RxCommand> Secplus1::decode_packet() const
        {
            CommandType cmd_type = to_CommandType(this->rx_state_.buffer[0], CommandType::UNKNOWN);
            return RxCommand { cmd_type, this->rx_state_.buffer[1] };
        }

        // ════════════════════════════════════════════════════════════════════
        //  Command Handling
        // ════════════════════════════════════════════════════════════════════

        void Secplus1::handle_command(const RxCommand& cmd)
        {
            switch (cmd.type) {
            case CommandType::TOGGLE_DOOR_RELEASE:
                // A wall panel starting signal – ignore to avoid disrupting emulation
                ESP_LOGD(TAG, "Wall panel starting signal ignored");
                break;

            case CommandType::QUERY_DOOR_STATUS:
                this->handle_door_status(cmd);
                break;

            case CommandType::QUERY_DOOR_STATUS_ALT:
                this->handle_alternate_query(cmd);
                break;

            case CommandType::QUERY_OTHER_STATUS:
                this->handle_other_status(cmd);
                break;

            case CommandType::OBSTRUCTION:
                this->handle_obstruction(cmd);
                break;

            case CommandType::TOGGLE_LIGHT_PRESS:
                // Motion detected or light toggle button pressed on the wall panel
                if (this->light_state_ == LightState::OFF) {
                    this->ratgdo_->received(MotionState::DETECTED);
                }
                break;

            case CommandType::TOGGLE_DOOR_PRESS:
                this->ratgdo_->received(ButtonState::PRESSED);
                break;

            default:
                break;
            }
        }

        // ── Door Status ────────────────────────────────────────────────────

        void Secplus1::handle_door_status(const RxCommand& cmd)
        {
            DoorState incoming = this->decode_door_state(cmd.data & DOOR_STATUS_MASK);

            // Always fire the once-callback so door_action sequences proceed
            if (this->unconfirmed_door_.value != incoming) {
                this->on_door_state_.trigger(incoming);
            }

            // On standard panels, require two consecutive identical readings
            if (!this->flags_.uses_alternate_query && !this->unconfirmed_door_.confirm(incoming)) {
                ESP_LOG1(TAG, "Door maybe %s, waiting for confirmation",
                    LOG_STR_ARG(DoorState_to_string(incoming)));
                return;
            }

            this->unconfirmed_door_.value = incoming;
            this->door_state_ = incoming;

            if (incoming == DoorState::STOPPED || incoming == DoorState::OPEN || incoming == DoorState::CLOSED) {
                this->flags_.door_moving = false;
            }

            this->ratgdo_->received(incoming);
        }

        void Secplus1::handle_alternate_query(const RxCommand& cmd)
        {
            this->flags_.uses_alternate_query = true;

            // On alternate panels, we can inject pending commands at the 0x37 boundary
            auto pending = this->peek_pending_tx();
            if (pending && pending.value() == CommandType::TOGGLE_LOCK_PRESS) {
                this->transmit_if_pending();
                return;
            }

            // Otherwise inject a door status query if the door is moving or if
            // enough time has elapsed since the last query
            if (this->flags_.door_moving || (millis() - this->last_status_query_time_ > STATUS_QUERY_INTERVAL_MS)) {
                this->send_byte(static_cast<uint8_t>(CommandType::QUERY_DOOR_STATUS));
                this->last_status_query_time_ = millis();
            }
        }

        // ── Light / Lock Status ────────────────────────────────────────────

        void Secplus1::handle_other_status(const RxCommand& cmd)
        {
            LightState incoming_light = to_LightState((cmd.data >> LIGHT_STATUS_BIT) & 1, LightState::UNKNOWN);

            if (this->flags_.uses_alternate_query || this->unconfirmed_light_.confirm(incoming_light)) {
                this->light_state_ = incoming_light;
                this->ratgdo_->received(incoming_light);
            }

            LockState incoming_lock = to_LockState((~cmd.data >> LOCK_STATUS_BIT) & 1, LockState::UNKNOWN);

            if (this->flags_.uses_alternate_query || this->unconfirmed_lock_.confirm(incoming_lock)) {
                this->lock_state_ = incoming_lock;
                this->ratgdo_->received(incoming_lock);
            }
        }

        // ── Obstruction ────────────────────────────────────────────────────

        void Secplus1::handle_obstruction(const RxCommand& cmd)
        {
            ObstructionState state = (cmd.data == 0) ? ObstructionState::CLEAR : ObstructionState::OBSTRUCTED;
            this->ratgdo_->received(state);
        }

        // ── Door State Decoding ────────────────────────────────────────────

        DoorState Secplus1::decode_door_state(uint8_t status_bits) const
        {
            switch (status_bits) {
            case DOOR_STATUS_OPEN:
                return DoorState::OPEN;
            case DOOR_STATUS_CLOSED:
                return DoorState::CLOSED;
            case DOOR_STATUS_STOPPED_A:
            case DOOR_STATUS_STOPPED_B:
                return DoorState::STOPPED;
            case DOOR_STATUS_OPENING:
                return DoorState::OPENING;
            case DOOR_STATUS_CLOSING:
                return DoorState::CLOSING;
            default:
                return DoorState::UNKNOWN;
            }
        }

        // ════════════════════════════════════════════════════════════════════
        //  Transmit Path
        // ════════════════════════════════════════════════════════════════════

        void Secplus1::schedule_tx(CommandType cmd, uint32_t send_at)
        {
            if (send_at == 0) {
                send_at = millis();
            }
            this->tx_queue_.push(ScheduledTx { cmd, send_at });
        }

        void Secplus1::schedule_press_release_pair(CommandType press_cmd)
        {
            uint32_t now = millis();
            switch (press_cmd) {
            case CommandType::TOGGLE_DOOR_PRESS:
                this->schedule_tx(CommandType::TOGGLE_DOOR_RELEASE, now + DOOR_RELEASE_DELAY_MS);
                break;
            case CommandType::TOGGLE_LIGHT_PRESS:
                this->schedule_tx(CommandType::TOGGLE_LIGHT_RELEASE, now + LIGHT_RELEASE_DELAY_MS);
                break;
            case CommandType::TOGGLE_LOCK_PRESS:
                this->schedule_tx(CommandType::TOGGLE_LOCK_RELEASE, now + LOCK_RELEASE_DELAY_MS);
                break;
            default:
                break;
            }
        }

        optional<CommandType> Secplus1::peek_pending_tx() const
        {
            if (this->tx_queue_.empty()) {
                return { };
            }
            auto& top = this->tx_queue_.top();
            if (top.send_at > millis()) {
                return { };
            }
            return top.command;
        }

        optional<CommandType> Secplus1::pop_pending_tx()
        {
            auto cmd = this->peek_pending_tx();
            if (cmd) {
                this->tx_queue_.pop();
            }
            return cmd;
        }

        bool Secplus1::transmit_if_pending()
        {
            auto cmd = this->pop_pending_tx();
            if (!cmd) {
                return false;
            }
            this->schedule_press_release_pair(cmd.value());
            this->send_byte(static_cast<uint8_t>(cmd.value()));
            return true;
        }

        void Secplus1::send_byte(uint8_t value)
        {
            // For status query commands (0x38, 0x39, 0x3A), the GDO responds
            // on the same wire, so we must keep the RX interrupt enabled.
            // For all other commands we disable it to avoid reading our own TX.
            bool is_query = (value == static_cast<uint8_t>(CommandType::QUERY_DOOR_STATUS))
                || (value == static_cast<uint8_t>(CommandType::OBSTRUCTION))
                || (value == static_cast<uint8_t>(CommandType::QUERY_OTHER_STATUS));

            if (!is_query) {
                this->sw_serial_.enableIntTx(false);
            }

            this->sw_serial_.write(value);
            this->last_tx_time_ = millis();

            if (!is_query) {
                this->sw_serial_.enableIntTx(true);
            }

            ESP_LOGD(TAG, "[%d] Sent byte: [%02X]", millis(), value);
        }

        // ════════════════════════════════════════════════════════════════════
        //  User Actions
        // ════════════════════════════════════════════════════════════════════

        void Secplus1::light_action(LightAction action)
        {
            ESP_LOG1(TAG, "Light action: %s", LOG_STR_ARG(LightAction_to_string(action)));
            if (action == LightAction::UNKNOWN) {
                return;
            }
            if (action == LightAction::TOGGLE
                || (action == LightAction::ON && this->light_state_ == LightState::OFF)
                || (action == LightAction::OFF && this->light_state_ == LightState::ON)) {
                this->send_light_toggle();
            }
        }

        void Secplus1::lock_action(LockAction action)
        {
            ESP_LOG1(TAG, "Lock action: %s", LOG_STR_ARG(LockAction_to_string(action)));
            if (action == LockAction::UNKNOWN) {
                return;
            }
            if (action == LockAction::TOGGLE
                || (action == LockAction::LOCK && this->lock_state_ == LockState::UNLOCKED)
                || (action == LockAction::UNLOCK && this->lock_state_ == LockState::LOCKED)) {
                this->send_lock_toggle();
            }
        }

        void Secplus1::door_action(DoorAction action)
        {
            ESP_LOG1(TAG, "Door action: %s, door state: %s",
                LOG_STR_ARG(DoorAction_to_string(action)),
                LOG_STR_ARG(DoorState_to_string(this->door_state_)));

            if (action == DoorAction::UNKNOWN) {
                return;
            }

            switch (action) {
            case DoorAction::TOGGLE:
                this->send_door_toggle();
                break;

            case DoorAction::OPEN:
                if (this->door_state_ == DoorState::CLOSED || this->door_state_ == DoorState::CLOSING) {
                    this->send_door_toggle();
                } else if (this->door_state_ == DoorState::STOPPED) {
                    // From stopped, a toggle starts closing. We need to reverse
                    // direction once it begins closing.
                    this->send_door_toggle();
                    this->on_door_state_([this](DoorState s) {
                        if (s == DoorState::CLOSING) {
                            // Reverse direction; on some openers this stops, on others it opens
                            this->send_door_toggle();
                            this->on_door_state_([this](DoorState s) {
                                if (s == DoorState::STOPPED) {
                                    this->send_door_toggle();
                                }
                            });
                        }
                    });
                }
                break;

            case DoorAction::CLOSE:
                if (this->door_state_ == DoorState::OPEN) {
                    this->send_door_toggle();
                } else if (this->door_state_ == DoorState::OPENING) {
                    // Toggle to stop, then toggle again to close
                    this->send_door_toggle();
                    this->on_door_state_([this](DoorState s) {
                        if (s == DoorState::STOPPED) {
                            this->send_door_toggle();
                        }
                    });
                } else if (this->door_state_ == DoorState::STOPPED) {
                    this->send_door_toggle();
                }
                break;

            case DoorAction::STOP:
                if (this->door_state_ == DoorState::OPENING) {
                    this->send_door_toggle();
                } else if (this->door_state_ == DoorState::CLOSING) {
                    // Toggle reverses to opening, then toggle again to stop
                    this->send_door_toggle();
                    this->on_door_state_([this](DoorState s) {
                        if (s == DoorState::OPENING) {
                            this->send_door_toggle();
                        }
                    });
                }
                break;

            default:
                break;
            }
        }

        // ── Toggle Helpers ─────────────────────────────────────────────────

        void Secplus1::send_door_toggle()
        {
            this->schedule_tx(CommandType::TOGGLE_DOOR_PRESS);
            this->schedule_tx(CommandType::QUERY_DOOR_STATUS);

            if (this->door_state_ == DoorState::STOPPED
                || this->door_state_ == DoorState::OPEN
                || this->door_state_ == DoorState::CLOSED) {
                this->flags_.door_moving = true;
            }
        }

        void Secplus1::send_light_toggle()
        {
            this->schedule_tx(CommandType::TOGGLE_LIGHT_PRESS);
        }

        void Secplus1::send_lock_toggle()
        {
            this->schedule_tx(CommandType::TOGGLE_LOCK_PRESS);
        }

        // ════════════════════════════════════════════════════════════════════
        //  Protocol Interface Stubs
        // ════════════════════════════════════════════════════════════════════

        Result Secplus1::call(Args args)
        {
            return { };
        }

    } // namespace secplus1
} // namespace ratgdo
} // namespace esphome

#endif // PROTOCOL_SECPLUSV1
