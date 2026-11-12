# One discrete, debounced safety-performance-indicator event.
#
# Published once per violation, when it CLEARS (after hysteresis), not
# continuously while it holds -- this is what turns high-volume runtime
# signals into the low-volume stream a safety argument can reference.

uint8 SEVERITY_INFO = 0
uint8 SEVERITY_WARN = 1
uint8 SEVERITY_VIOLATION = 2

std_msgs/Header header         # event end (cleared) time
builtin_interfaces/Time start_stamp  # event start (triggered) time

string spi_id                  # e.g. "SPI-1", "SPI-2", "SPI-3", "SPI-4"
string claim_id                # safety-case claim this event evidences
string zone_id                 # zone instance name, "" if not zone-scoped
string zone_type                # "free" | "speed" | "keepout" | ""

float64 peak_value             # peak observed value over the event
float64 threshold               # threshold that was exceeded
uint8 severity

string config_id               # e.g. "base", "native_keepout", "native_speed", "external"
string run_id

geometry_msgs/Point position    # robot position at the peak
