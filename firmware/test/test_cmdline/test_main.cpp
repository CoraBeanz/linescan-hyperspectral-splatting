// Command-line parsing, number parsing and reply formatting.
#include <unity.h>

#include <string.h>

#include "sm_cmdline.h"
#include "sm_config.h"

using sm::Args;
using sm::LineBuf;

void setUp() {}
void tearDown() {}

void test_parse_verb_and_pairs() {
  char line[] = "  scan Mode=stare start=-106 PERIOD=33333.333  lines=213 id=42\r";
  Args a;
  TEST_ASSERT_TRUE(a.parse(line));
  TEST_ASSERT_EQUAL_STRING("SCAN", a.verb());
  TEST_ASSERT_EQUAL_STRING("42", a.id());
  TEST_ASSERT_EQUAL(4, a.count());
  TEST_ASSERT_TRUE(a.has("mode"));
  TEST_ASSERT_TRUE(a.has("period"));
  int32_t start = 0, lines = 0, missing = 7;
  TEST_ASSERT_TRUE(a.getI32("start", &start, -1000, 1000));
  TEST_ASSERT_TRUE(a.getI32("lines", &lines, 1, 1000));
  TEST_ASSERT_TRUE(a.getI32("step", &missing, -5, 5));  // absent: untouched
  TEST_ASSERT_EQUAL_INT32(-106, start);
  TEST_ASSERT_EQUAL_INT32(213, lines);
  TEST_ASSERT_EQUAL_INT32(7, missing);
  uint64_t period = 0;
  TEST_ASSERT_TRUE(a.getQ16("period", &period, 1000, 60000000));
  TEST_ASSERT_EQUAL_UINT64(33333ull * 65536 + 21823, period);
  const char* mode = nullptr;
  TEST_ASSERT_TRUE(a.getStr("mode", &mode));
  TEST_ASSERT_EQUAL_STRING("stare", mode);
  TEST_ASSERT_NULL(a.unused());
}

void test_unused_key_is_reported() {
  char line[] = "MOVE pos=10 sped=100";
  Args a;
  TEST_ASSERT_TRUE(a.parse(line));
  int32_t pos = 0;
  TEST_ASSERT_TRUE(a.getI32("pos", &pos, -100, 100));
  TEST_ASSERT_EQUAL_STRING("sped", a.unused());
}

void test_syntax_errors() {
  Args a;
  char l1[] = "MOVE 100";
  TEST_ASSERT_FALSE(a.parse(l1));
  char l2[] = "MOVE pos=1 pos=2";
  TEST_ASSERT_FALSE(a.parse(l2));
  char l3[] = "MOVE =5";
  TEST_ASSERT_FALSE(a.parse(l3));
  char l4[] = "   ";
  TEST_ASSERT_FALSE(a.parse(l4));
  char l5[] = "CFG a=1 b=2 c=3 d=4 e=5 f=6 g=7 h=8 i=9 j=10 k=11 l=12 m=13 n=14 o=15 p=16 q=17";
  TEST_ASSERT_FALSE(a.parse(l5));
  TEST_ASSERT_EQUAL_STRING("CFG", a.verb());
  // The reply to a bad line still names its verb and carries its id.
  char l6[] = "move pos=1 pos=2 ID=9";
  TEST_ASSERT_FALSE(a.parse(l6));
  TEST_ASSERT_EQUAL_STRING("MOVE", a.verb());
  TEST_ASSERT_EQUAL_STRING("9", a.id());
  TEST_ASSERT_NOT_NULL(strstr(a.error(), "repeated key pos"));
  char l7[] = "scan 5 id=x";
  TEST_ASSERT_FALSE(a.parse(l7));
  TEST_ASSERT_EQUAL_STRING("x", a.id());
}

void test_bad_values() {
  Args a;
  char line[] = "MOVE pos=12x v=99999 period=1.5.2";
  TEST_ASSERT_TRUE(a.parse(line));
  int32_t v = 0;
  TEST_ASSERT_FALSE(a.getI32("pos", &v, -100, 100));
  TEST_ASSERT_NOT_NULL(strstr(a.error(), "pos"));
  TEST_ASSERT_FALSE(a.getI32("v", &v, 1, 10000));
  TEST_ASSERT_NOT_NULL(strstr(a.error(), "range"));
  uint64_t q = 0;
  TEST_ASSERT_FALSE(a.getQ16("period", &q, 1000, 60000000));
}

void test_parse_numbers() {
  int64_t v = 0;
  TEST_ASSERT_TRUE(sm::parseI64("-9223372036854775807", &v));
  TEST_ASSERT_TRUE(v == -9223372036854775807LL);
  TEST_ASSERT_FALSE(sm::parseI64("9223372036854775808", &v));
  TEST_ASSERT_FALSE(sm::parseI64("", &v));
  TEST_ASSERT_FALSE(sm::parseI64("-", &v));
  TEST_ASSERT_FALSE(sm::parseI64("1e3", &v));
  TEST_ASSERT_TRUE(sm::parseI64("+15", &v));
  TEST_ASSERT_EQUAL_INT64(15, v);

  uint64_t q = 0;
  TEST_ASSERT_TRUE(sm::parseQ16("1000", &q));
  TEST_ASSERT_EQUAL_UINT64(1000ull * 65536, q);
  TEST_ASSERT_TRUE(sm::parseQ16("0.5", &q));
  TEST_ASSERT_EQUAL_UINT64(32768, q);
  TEST_ASSERT_TRUE(sm::parseQ16("16666.666667", &q));
  TEST_ASSERT_EQUAL_UINT64(16666ull * 65536 + 43691, q);
  TEST_ASSERT_TRUE(sm::parseQ16(".25", &q));
  TEST_ASSERT_EQUAL_UINT64(16384, q);
  TEST_ASSERT_FALSE(sm::parseQ16("-5", &q));
  TEST_ASSERT_FALSE(sm::parseQ16("1.1234567", &q));
  TEST_ASSERT_FALSE(sm::parseQ16(".", &q));
  TEST_ASSERT_FALSE(sm::parseQ16("12a", &q));
}

void test_linebuf_formatting() {
  LineBuf b;
  b.word("EV").word("LINE").kv("n", 0).kv("t", 1234567890123LL).kv("pos", -106);
  TEST_ASSERT_EQUAL_STRING("EV LINE n=0 t=1234567890123 pos=-106", b.c_str());
  b.clear();
  b.kv("min", INT64_MIN).kvHex("ver", 0x21).kvHex("raw", 0).kvHex("big", 0xC0000000u);
  TEST_ASSERT_EQUAL_STRING("min=-9223372036854775808 ver=0x21 raw=0x0 big=0xc0000000", b.c_str());
  b.clear();
  b.kvQ16("period", 33333ull * 65536 + 21823).kvQ16("p2", 1000ull * 65536).kvQ16("p3", 65535);
  TEST_ASSERT_EQUAL_STRING("period=33333.333 p2=1000.000 p3=1.000", b.c_str());
  b.clear();
  b.word("ERR").word("MOVE").kvStr("code", "range").msg("target 4000 is outside min..max");
  TEST_ASSERT_EQUAL_STRING("ERR MOVE code=range msg=target 4000 is outside min..max", b.c_str());
}

void test_linebuf_truncates_safely() {
  LineBuf b;
  for (int i = 0; i < 200; ++i) b.kv("key", 123456);
  TEST_ASSERT_EQUAL(LineBuf::kCap, b.size());
  TEST_ASSERT_EQUAL(LineBuf::kCap, strlen(b.c_str()));
}

void test_config_defaults_are_valid() {
  sm::Config c;
  sm::configDefaults(&c);
  const sm::ConfigKey* bad = nullptr;
  TEST_ASSERT_NULL(sm::configProblem(c, &bad));
  TEST_ASSERT_EQUAL_INT32(32, c.usteps);
  TEST_ASSERT_EQUAL_INT32(200, c.irun);
  TEST_ASSERT_EQUAL_INT32(-711, c.home_pos);
  c.usteps = 24;
  TEST_ASSERT_NOT_NULL(sm::configProblem(c, &bad));
  TEST_ASSERT_EQUAL_STRING("usteps", bad->name);
  sm::configDefaults(&c);
  c.vstart = c.vmax + 1;
  TEST_ASSERT_NOT_NULL(sm::configProblem(c, &bad));
  TEST_ASSERT_NULL(bad);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_parse_verb_and_pairs);
  RUN_TEST(test_unused_key_is_reported);
  RUN_TEST(test_syntax_errors);
  RUN_TEST(test_bad_values);
  RUN_TEST(test_parse_numbers);
  RUN_TEST(test_linebuf_formatting);
  RUN_TEST(test_linebuf_truncates_safely);
  RUN_TEST(test_config_defaults_are_valid);
  return UNITY_END();
}
