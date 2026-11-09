"""Prepare ONLY a new experimental copy; preserve historical proposals/evidence."""
import difflib
from pathlib import Path
import sys

root = Path(sys.argv[1])
mode = sys.argv[2]
cpp = root / 'libraries/SITL/SIM_JSON.cpp'
header = root / 'libraries/SITL/SIM_JSON.h'
old = cpp.read_text()
if mode == 'instrument':
    assert 'GNSS_EXPERIMENT_TRACE' not in old
    new = old.replace('#include <errno.h>', '#include <errno.h>\n#include <time.h>')
    anchor = '    last_timestamp_s = state.timestamp_s;\n    frame_counter++;'
    trace = '''    // Experiment only: observe applied physics state, never send navigation.
    static FILE *trace = []() -> FILE * {
        const char *path = getenv("GNSS_EXPERIMENT_TRACE");
        return path ? fopen(path, "wx") : nullptr;
    }();
    if (trace != nullptr && frame_counter % 20 == 0) {
        struct timespec wall{};
        clock_gettime(CLOCK_MONOTONIC, &wall);
        fprintf(trace, "%.9f,%u,%u,%.9f,%llu,%llu,%llu,%llu,%.6f\\n",
                state.timestamp_s, unsigned(state.no_time_sync), unsigned(use_time_sync),
                deltat, (unsigned long long)time_now_us,
                (unsigned long long)AP_HAL::micros64(),
                (unsigned long long)wall.tv_sec * 1000000000ULL + wall.tv_nsec,
                (unsigned long long)frame_time_us, double(rate_hz));
        fflush(trace);
    }
'''
    assert new.count(anchor) == 1
    new = new.replace(anchor, trace + anchor)
    cpp.write_text(new)
    # Test-only access, without making the parser a public flight API.
    text = header.read_text()
    assert text.count('private:') == 1
    header.write_text(text.replace('private:', 'private:\n    friend class JSONParserTestAccess;'))
elif mode == 'fix':
    anchor = '''            case BOOLEAN:
                *((bool *)key.ptr) = strtoull(p, nullptr, 10) != 0;
                //printf("%s/%s = %i\\n", key.section, key.key, *((unit8_t *)key.ptr));
                break;'''
    replacement = '''            case BOOLEAN: {
                // Exact JSON literals, plus legacy signed/unsigned decimal integers.
                auto whitespace = [](char c) {
                    return c == ' ' || c == '\\t' || c == '\\r' || c == '\\n';
                };
                while (whitespace(*p)) {
                    ++p;
                }
                const char *end = p;
                bool value;
                if (strncmp(p, "true", 4) == 0) {
                    end = p + 4;
                    value = true;
                } else if (strncmp(p, "false", 5) == 0) {
                    end = p + 5;
                    value = false;
                } else {
                    const char *digits = p + ((*p == '+' || *p == '-') ? 1 : 0);
                    if (*digits < '0' || *digits > '9') {
                        printf("Invalid JSON boolean for %s\\n", key.key);
                        return 0;
                    }
                    char *numeric_end = nullptr;
                    errno = 0;
                    const auto number = strtoull(p, &numeric_end, 10);
                    if (errno == ERANGE || numeric_end == p) {
                        printf("Invalid JSON boolean integer for %s\\n", key.key);
                        return 0;
                    }
                    end = numeric_end;
                    value = number != 0;
                }
                while (whitespace(*end)) {
                    ++end;
                }
                if (*end != ',' && *end != '}' && *end != '\\0') {
                    printf("Invalid JSON boolean suffix for %s\\n", key.key);
                    return 0;
                }
                *((bool *)key.ptr) = value;
                break;
            }'''
    assert old.count(anchor) == 1
    new = old.replace(anchor, replacement)
    # Optional field has a documented per-packet default, not sticky prior state.
    anchor2 = '    uint32_t received_bitmask = 0;'
    assert new.count(anchor2) == 1
    new = new.replace(anchor2, anchor2 + '\n    state.no_time_sync = false;')
    cpp.write_text(new)
    patch = ''.join(difflib.unified_diff(old.splitlines(True), new.splitlines(True),
                    fromfile='a/libraries/SITL/SIM_JSON.cpp',
                    tofile='b/libraries/SITL/SIM_JSON.cpp'))
    (Path(__file__).parent / 'AP444_BOOLEAN_EXACT.patch').write_text(patch)
else:
    raise ValueError(mode)
