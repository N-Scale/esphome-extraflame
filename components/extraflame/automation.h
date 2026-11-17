#pragma once

#include <algorithm>
#include <utility>

#include "extraflame.h"
#include "esphome/core/automation.h"

namespace esphome
{
  namespace extraflame
  {
    static const char *TAG = "extraflame.hub";

    static bool is_valid_byte(int64_t value, const char *name)
    {
      if (value < 0x00 || value > 0xFF)
      {
        ESP_LOGE(TAG, "'%s' must be between 0 and 255, got %lld", name, static_cast<long long>(value));
        return false;
      }

      return true;
    }

    template <typename... Ts>
    class ExtraflameWriteAction : public Action<Ts...>, public Parented<ExtraflameHub>
    {
    public:
      TEMPLATABLE_VALUE(int64_t, address)
      TEMPLATABLE_VALUE(int64_t, value)

      void set_memory(const uint8_t memory) { this->memory_ = memory; }
      void set_expected_values(std::vector<uint8_t> values) { this->expected_values_ = std::move(values); }

      void play_complex(Ts... x) override
      {
        this->num_running_++;

        const auto raw_address = this->address_.value(x...);
        const auto raw_value = this->value_.value(x...);
        if (!is_valid_byte(raw_address, "address") || !is_valid_byte(raw_value, "value"))
        {
          this->stop_complex();
          return;
        }
        const auto address = static_cast<uint8_t>(raw_address);
        const auto value_write = static_cast<uint8_t>(raw_value);

        auto request = ExtraflameRequest{
            .command = {this->memory_, address},
            .on_response = [this, address, value_write, x...](uint8_t value, bool success) {
              if (!success)
              {
                this->stop_complex();
                return;
              }

              if (!this->expected_values_.empty() &&
                  std::find(this->expected_values_.begin(), this->expected_values_.end(), value) ==
                      this->expected_values_.end())
              {
                ESP_LOGW(TAG, "Write to 0x%02X:0x%02X skipped: unexpected current value 0x%02X",
                         this->memory_, address, value);
                this->stop_complex();
                return;
              }

              if (value_write == value)
              {
                // values are the same, we don't need an update
                this->play_next_(x...);
              }
              else
              {
                uint8_t write_memory = 0x80 + this->memory_;
                uint8_t checksum = (write_memory + address + value_write) & 0xFF;
                auto write_request = ExtraflameRequest{
                    .command = {write_memory, address, value_write, checksum},
                    .on_response = [this, x...](uint8_t value, bool success) {
                      if (success)
                      {
                        this->play_next_(x...);
                      }
                      else
                      {
                        this->stop_complex();
                      }
                    }};
                this->parent_->add_request(write_request, true);
              }
            }};
        this->parent_->add_request(request, true);
      }

      void play(Ts... x) override { this->play_complex(x...); }

    protected:
      uint8_t memory_{};
      std::vector<uint8_t> expected_values_;
    };

#ifdef USE_EXTRAFLAME_DUMP
    template <typename... Ts>
    class ExtraflameDumpAction : public Action<Ts...>, public Parented<ExtraflameHub>
    {
    public:
      TEMPLATABLE_VALUE(int64_t, start)
      TEMPLATABLE_VALUE(int64_t, end)

      void set_memory(const uint8_t memory) { this->memory_ = memory; }

      void register_finish_trigger(ExtraflameDumpFinishTrigger *trigger) { this->finish_triggers_.push_back(trigger); }

      void play_complex(Ts... x) override
      {
        this->num_running_++;

        auto start = this->start_.value(x...);
        auto end = this->end_.value(x...);
        if (!is_valid_byte(start, "start") || !is_valid_byte(end, "end"))
        {
          this->stop_complex();
          return;
        }

        this->parent_->start_dumping(this->memory_, static_cast<uint8_t>(start), static_cast<uint8_t>(end),
                                     this->finish_triggers_);

        this->play_next_(x...);
      }

      void play(Ts... x) override { this->play_complex(x...); }

    protected:
      uint8_t memory_{};
      std::vector<ExtraflameDumpFinishTrigger *> finish_triggers_;
    };


    template<typename... Ts>
    class ExtraflameIsDumpingCondition : public Condition<Ts...>, public Parented<ExtraflameHub> {
    public:
      bool check(Ts... x) override { return this->parent_->is_dumping(); }
    };

#endif

  } // namespace extraflame
} // namespace esphome
