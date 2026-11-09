#include <AP_gtest.h>
#include <SITL/SIM_JSON.h>
#include <string>
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace SITL {
class JSONParserTestAccess {
public:
    static uint32_t parse(JSON &j, const std::string &s) { return j.parse_sensors(s.c_str()); }
    static bool value(const JSON &j) { return j.state.no_time_sync; }
};
}

static std::string packet(const std::string &token, bool present=true)
{
    return "{\"timestamp\":1.25,\"imu\":{\"gyro\":[0,0,0],\"accel_body\":[0,0,-9.8]},"
           "\"position\":[0,0,0],\"quaternion\":[1,0,0,0],\"velocity\":[0,0,0]" +
           (present ? ",\"no_time_sync\":" + token : "") + "}";
}

TEST(JSONParser, ExactLiteralsAndWhitespace) {
    SITL::JSON j("JSON");
    for (const auto &t : {"true", " true ", "\t\r\ntrue\n\r\t"}) {
        ASSERT_NE(SITL::JSONParserTestAccess::parse(j,packet(t)), 0U);
        EXPECT_TRUE(SITL::JSONParserTestAccess::value(j));
    }
    for (const auto &t : {"false", " \t\r\nfalse\t "}) {
        ASSERT_NE(SITL::JSONParserTestAccess::parse(j,packet(t)), 0U);
        EXPECT_FALSE(SITL::JSONParserTestAccess::value(j));
    }
}
TEST(JSONParser, LegacyDecimalIntegers) {
    SITL::JSON j("JSON");
    for (const auto &t : {"1", "2", "+1", "-1", "18446744073709551615"}) {
        ASSERT_NE(SITL::JSONParserTestAccess::parse(j,packet(t)), 0U);
        EXPECT_TRUE(SITL::JSONParserTestAccess::value(j));
    }
    for (const auto &t : {"0", "-0", "+0", " 000 "}) {
        ASSERT_NE(SITL::JSONParserTestAccess::parse(j,packet(t)), 0U);
        EXPECT_FALSE(SITL::JSONParserTestAccess::value(j));
    }
}
TEST(JSONParser, RejectMalformedTokens) {
    SITL::JSON j("JSON");
    for (const auto &t : {"trueXYZ", "false0", "\"true\"", "null", "TRUE", "", " ",
                          "1.0", "1e0", "0x1", "1garbage", "+", "--1",
                          "18446744073709551616", "true false", "\vtrue"}) {
        EXPECT_EQ(SITL::JSONParserTestAccess::parse(j,packet(t)), 0U) << t;
    }
}
TEST(JSONParser, MissingOptionalFieldResetsToDefault) {
    SITL::JSON j("JSON");
    ASSERT_NE(SITL::JSONParserTestAccess::parse(j,packet("true")), 0U);
    EXPECT_TRUE(SITL::JSONParserTestAccess::value(j));
    ASSERT_NE(SITL::JSONParserTestAccess::parse(j,packet("",false)), 0U);
    EXPECT_FALSE(SITL::JSONParserTestAccess::value(j));
}
TEST(JSONParser, RejectMissingRequiredField) {
    SITL::JSON j("JSON");
    EXPECT_EQ(SITL::JSONParserTestAccess::parse(j,"{\"no_time_sync\":true}"), 0U);
}
AP_GTEST_MAIN()
