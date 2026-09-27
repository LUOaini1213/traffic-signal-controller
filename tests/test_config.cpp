#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "helpers.hpp"

using namespace tsc;
using namespace tsc::test;
using nlohmann::json;

namespace {

// Expect parse_config to reject `j` with at least one message containing `needle`.
void expect_rejected(const json& j, const std::string& needle) {
  try {
    (void)parse_config(j);
    FAIL() << "config accepted, expected an error containing: " << needle;
  } catch (const ConfigError& e) {
    bool found = false;
    for (const auto& msg : e.errors()) found = found || msg.find(needle) != std::string::npos;
    EXPECT_TRUE(found) << "expected '" << needle << "' in:\n" << e.what();
  }
}

json edited(const std::function<void(json&)>& f) {
  auto j = repo_config_json();
  f(j);
  return j;
}

}  // namespace

TEST(Config, RepositoryConfigIsValid) {
  const Config c = repo_config();
  EXPECT_EQ(c.groups.size(), 8u);
  EXPECT_EQ(c.phases.size(), 4u);
  EXPECT_EQ(c.detectors.size(), 16u);
  EXPECT_EQ(c.tick, 500);
  EXPECT_EQ(c.phases[NS_T].min_green, 10'000);
  EXPECT_EQ(c.phases[NS_T].max_green, 45'000);
  EXPECT_EQ(c.phases[NS_R].passage, 2'500);
  EXPECT_EQ(c.phases[NS_T].recall, Recall::Min);
  EXPECT_EQ(c.phases[NS_R].recall, Recall::None);
  EXPECT_EQ(c.group_phase[G_ER], EW_R);
  EXPECT_EQ(c.group_phase[G_ST], NS_T);
  EXPECT_TRUE(c.conflict(G_NT, G_SR));
  EXPECT_FALSE(c.conflict(G_NT, G_ST));
  EXPECT_EQ(c.detectors[c.detector_index("W_2")].phase, EW_R);
  EXPECT_TRUE(c.detectors[c.detector_index("W_2")].extends);
  EXPECT_FALSE(c.detectors[c.detector_index("W_2s")].extends);  // stop-bar loop: call only
  EXPECT_EQ(c.detectors[c.detector_index("W_2s")].approach, "W");
  EXPECT_EQ(c.faults.stuck_off, 900'000);
  EXPECT_EQ(c.faults.max_faulty, 4u);
  EXPECT_EQ(c.faults.stuck_off_min_others, 20u);
  EXPECT_TRUE(c.sumo.is_object());
}

TEST(Config, MinGreenGreaterThanMaxGreenIsRejected) {
  expect_rejected(edited([](json& j) { j["phases"][0]["min_green_s"] = 50; }),
                  "min_green_s (50) is greater than max_green_s (45)");
}

TEST(Config, MinGreenEqualToMaxGreenIsAccepted) {
  EXPECT_NO_THROW(parse_config(edited([](json& j) { j["phases"][0]["min_green_s"] = 45; })));
}

TEST(Config, AsymmetricConflictMatrixIsRejected) {
  expect_rejected(edited([](json& j) { j["conflicts"][0][3] = 0; }), "not symmetric ('N_T' vs 'S_R')");
}

TEST(Config, SelfConflictIsRejected) {
  expect_rejected(edited([](json& j) { j["conflicts"][2][2] = 1; }), "'N_R' conflicts with itself");
}

TEST(Config, WrongMatrixShapeIsRejected) {
  expect_rejected(edited([](json& j) { j["conflicts"].erase(7); }), "expected 8 rows, got 7");
  expect_rejected(edited([](json& j) { j["conflicts"][4].erase(0); }), "row 4 has 7 entries");
}

TEST(Config, NonBinaryConflictEntryIsRejected) {
  expect_rejected(edited([](json& j) { j["conflicts"][0][1] = 2; }), "entries must be 0 or 1");
}

TEST(Config, ConflictingGroupsInOnePhaseAreRejected) {
  expect_rejected(edited([](json& j) {
                    j["phases"][0]["groups"] = {"N_T", "S_T", "S_R"};
                    j["phases"][1]["groups"] = {"N_R"};
                  }),
                  "'N_T' and 'S_R' conflict");
}

TEST(Config, GroupServedTwiceOrNeverIsRejected) {
  expect_rejected(edited([](json& j) { j["phases"][1]["groups"] = {"N_R", "S_R", "N_T"}; }),
                  "'N_T' appears in more than one phase");
  expect_rejected(edited([](json& j) { j["phases"][1]["groups"] = {"N_R"}; }),
                  "'S_R' is not served by any phase");
}

TEST(Config, UnknownNamesAreRejected) {
  expect_rejected(edited([](json& j) { j["phases"][0]["groups"] = {"N_T", "X"}; }), "unknown group 'X'");
  expect_rejected(edited([](json& j) { j["detectors"][0]["phase"] = "nope"; }), "unknown phase 'nope'");
  expect_rejected(edited([](json& j) { j["phases"][0]["min_gren_s"] = 5; }), "unknown key 'min_gren_s'");
  expect_rejected(edited([](json& j) { j["colour"] = "red"; }), "unknown key 'colour'");
  expect_rejected(edited([](json& j) { j["detectors"][0]["mode"] = "hold"; }), "expected \"extend\" or \"call\"");
  expect_rejected(edited([](json& j) { j["detectors"][0]["approach"] = 3; }), "approach: expected a string");
}

TEST(Config, MissingKeysAreRejected) {
  expect_rejected(edited([](json& j) { j["phases"][2].erase("yellow_s"); }), "missing required key 'yellow_s'");
  expect_rejected(edited([](json& j) { j.erase("conflicts"); }), "missing required key 'conflicts'");
}

TEST(Config, ClearanceTimesAreBounded) {
  expect_rejected(edited([](json& j) { j["phases"][1]["yellow_s"] = 2.5; }), "yellow_s must be in [3, 6]");
  expect_rejected(edited([](json& j) { j["phases"][1]["yellow_s"] = 6.5; }), "yellow_s must be in [3, 6]");
  expect_rejected(edited([](json& j) { j["phases"][3]["all_red_s"] = 0; }), "all_red_s must be in [tick, 6]");
  EXPECT_NO_THROW(parse_config(edited([](json& j) { j["phases"][3]["all_red_s"] = 0.5; })));
  EXPECT_NO_THROW(parse_config(edited([](json& j) { j["phases"][1]["yellow_s"] = 3; })));
}

TEST(Config, TimingMustBePositiveAndOnTheTick) {
  expect_rejected(edited([](json& j) { j["phases"][0]["min_green_s"] = 0; }), "min_green_s must be > 0");
  expect_rejected(edited([](json& j) { j["phases"][0]["passage_s"] = 0; }), "passage_s must be > 0");
  expect_rejected(edited([](json& j) { j["phases"][0]["passage_s"] = 3.2; }), "multiples of tick_ms");
  expect_rejected(edited([](json& j) { j["phases"][0]["passage_s"] = 50; }), "passage_s must not exceed max_green_s");
  expect_rejected(edited([](json& j) { j["tick_ms"] = 0; }), "tick_ms must be in (0, 1000]");
  expect_rejected(edited([](json& j) { j["tick_ms"] = 0.5; }), "expected an integer number of milliseconds");
  expect_rejected(edited([](json& j) { j["phases"][0]["max_green_s"] = "long"; }), "expected a number of seconds");
}

TEST(Config, PhaseThatCanNeverBeServedIsRejected) {
  expect_rejected(edited([](json& j) {
                    json keep = json::array();
                    for (const auto& d : j["detectors"]) {
                      if (d["phase"] != "EW_right") keep.push_back(d);
                    }
                    j["detectors"] = keep;
                  }),
                  "phase 'EW_right': recall is 'none' but no detector calls it");
}

TEST(Config, DuplicatesAreRejected) {
  expect_rejected(edited([](json& j) { j["detectors"][1]["id"] = "N_0"; }), "duplicate id 'N_0'");
  expect_rejected(edited([](json& j) { j["phases"][1]["name"] = "NS_through"; }), "duplicate phase name");
}

TEST(Config, FaultSettingsAreParsedAndChecked) {
  const Config c = parse_config(edited([](json& j) {
    j["faults"] = {{"enabled", false}, {"stuck_on_s", 120}, {"stuck_off_s", 60}, {"fallback_recall", "min"}};
  }));
  EXPECT_FALSE(c.faults.enabled);
  EXPECT_EQ(c.faults.stuck_on, 120'000);
  EXPECT_EQ(c.faults.stuck_off, 60'000);
  EXPECT_EQ(c.faults.fallback, Recall::Min);
  expect_rejected(edited([](json& j) { j["faults"]["fallback_recall"] = "none"; }), "must be 'min' or 'max'");
  expect_rejected(edited([](json& j) { j["faults"]["stuck_on_s"] = 0; }), "stuck thresholds must be > 0");
  expect_rejected(edited([](json& j) { j["faults"]["stuck_off_min_others"] = -1; }), "expected a non-negative integer");
  expect_rejected(edited([](json& j) { j["phases"][0]["recall"] = "always"; }), "expected one of");
}

TEST(Config, AllProblemsAreReportedTogether) {
  try {
    (void)parse_config(edited([](json& j) {
      j["phases"][0]["min_green_s"] = 60;
      j["conflicts"][0][3] = 0;
      j["phases"][2]["yellow_s"] = 1;
    }));
    FAIL() << "accepted";
  } catch (const ConfigError& e) {
    EXPECT_GE(e.errors().size(), 3u) << e.what();
  }
}

TEST(Config, LoadConfigReportsUnreadableAndMalformedFiles) {
  EXPECT_THROW((void)load_config("/nonexistent/config.json"), ConfigError);
  const auto path = std::filesystem::temp_directory_path() / "tsc_bad_config.json";
  {
    std::ofstream out(path);
    out << "{ \"name\": ";
  }
  EXPECT_THROW((void)load_config(path), ConfigError);
  std::filesystem::remove(path);
  EXPECT_NO_THROW((void)load_config(std::string(TSC_SOURCE_DIR) + "/configs/intersection.json"));
}

TEST(Config, IndexLookupsThrowOnUnknownNames) {
  const Config c = repo_config();
  EXPECT_EQ(c.group_index("W_R"), G_WR);
  EXPECT_EQ(c.phase_index("EW_through"), EW_T);
  EXPECT_THROW((void)c.group_index("Q"), std::out_of_range);
  EXPECT_THROW((void)c.detector_index("Q"), std::out_of_range);
}
