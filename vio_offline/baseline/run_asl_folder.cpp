// OpenVINS on a EuRoC/ASL-style folder, no ROS.
//
//   run_asl_folder <estimator_config.yaml> <dataset_root> <out_dir> [--start S] [--end S] [--scale F]
//
// Reads <root>/mav0/cam0/data.csv (+ data/<file>) and <root>/mav0/imu0/data.csv
// ("t_ns, wx, wy, wz, ax, ay, az", the layout written by dataset/recorder.py and
// by the EuRoC dataset). Measurements are fed in time order exactly like the
// OpenVINS serial ROS node does: every IMU sample first, a camera frame as soon
// as the IMU stream has passed its (offset-corrected) timestamp.
//
// Writes <out_dir>/openvins_states.csv: one row per processed frame once the
// filter is initialised. Pose is IMU-in-global (OpenVINS global frame: z up,
// arbitrary yaw). --start/--end are seconds from the first camera frame.
// --scale resizes images before tracking (the config intrinsics must match).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

struct ImuRow {
  int64_t t_ns;
  double w[3], a[3];
};
struct CamRow {
  int64_t t_ns;
  std::string file;
};

static std::string trim(const std::string &s) {
  size_t b = s.find_first_not_of(" \t\r\n");
  size_t e = s.find_last_not_of(" \t\r\n");
  return b == std::string::npos ? "" : s.substr(b, e - b + 1);
}

static std::vector<std::string> split_csv(const std::string &line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ','))
    out.push_back(trim(cell));
  return out;
}

static bool numeric_start(const std::string &line) {
  std::string s = trim(line);
  return !s.empty() && (std::isdigit(static_cast<unsigned char>(s[0])) || s[0] == '-');
}

int main(int argc, char **argv) {
  if (argc < 4) {
    std::fprintf(stderr, "usage: %s <estimator_config.yaml> <dataset_root> <out_dir> [--start S] [--end S] [--scale F]\n", argv[0]);
    return 2;
  }
  std::string config = argv[1], root = argv[2], out_dir = argv[3];
  double start_s = 0.0, end_s = -1.0, scale = 1.0;
  for (int i = 4; i + 1 < argc; i += 2) {
    std::string k = argv[i];
    if (k == "--start") start_s = std::atof(argv[i + 1]);
    else if (k == "--end") end_s = std::atof(argv[i + 1]);
    else if (k == "--scale") scale = std::atof(argv[i + 1]);
    else { std::fprintf(stderr, "unknown option %s\n", k.c_str()); return 2; }
  }

  // ---- inputs ----
  std::vector<ImuRow> imu;
  {
    std::ifstream f(root + "/mav0/imu0/data.csv");
    if (!f) { std::fprintf(stderr, "no IMU csv under %s\n", root.c_str()); return 3; }
    std::string line;
    while (std::getline(f, line)) {
      if (!numeric_start(line)) continue;
      auto c = split_csv(line);
      if (c.size() < 7) continue;
      ImuRow r;
      r.t_ns = std::stoll(c[0]);
      for (int k = 0; k < 3; k++) { r.w[k] = std::stod(c[1 + k]); r.a[k] = std::stod(c[4 + k]); }
      imu.push_back(r);
    }
  }
  std::vector<CamRow> cam;
  {
    std::ifstream f(root + "/mav0/cam0/data.csv");
    if (!f) { std::fprintf(stderr, "no camera csv under %s\n", root.c_str()); return 3; }
    std::string line;
    while (std::getline(f, line)) {
      if (!numeric_start(line)) continue;
      auto c = split_csv(line);
      if (c.size() < 2) continue;
      cam.push_back({std::stoll(c[0]), c[1]});
    }
  }
  std::stable_sort(imu.begin(), imu.end(), [](const ImuRow &a, const ImuRow &b) { return a.t_ns < b.t_ns; });
  std::stable_sort(cam.begin(), cam.end(), [](const CamRow &a, const CamRow &b) { return a.t_ns < b.t_ns; });
  if (imu.empty() || cam.empty()) { std::fprintf(stderr, "empty IMU or camera stream\n"); return 3; }

  // Seconds are counted from the first camera frame: keeps doubles precise for
  // any clock (UNIX ns, CLOCK_MONOTONIC ns) and matches the Python tools.
  const int64_t t0_ns = cam.front().t_ns;
  auto sec = [t0_ns](int64_t t) { return static_cast<double>(t - t0_ns) * 1e-9; };

  // ---- estimator ----
  auto parser = std::make_shared<ov_core::YamlParser>(config);
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);
  ov_msckf::VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = false;
  if (!parser->successful()) { std::fprintf(stderr, "config did not parse: %s\n", config.c_str()); return 4; }
  auto sys = std::make_shared<ov_msckf::VioManager>(params);

  std::ofstream out(out_dir + "/openvins_states.csv");
  if (!out) { std::fprintf(stderr, "cannot write to %s\n", out_dir.c_str()); return 5; }
  out << "t_s,t_ns,px,py,pz,qx,qy,qz,qw,vx,vy,vz,"
         "cov_pxx,cov_pxy,cov_pxz,cov_pyy,cov_pyz,cov_pzz,cov_rx,cov_ry,cov_rz,"
         "dt_cam_imu,n_slam,n_msckf,proc_ms\n";
  out.precision(12);

  size_t ci = 0;
  while (ci < cam.size() && sec(cam[ci].t_ns) < start_s) ci++;
  size_t n_frames = 0, n_states = 0, n_missing = 0;
  double proc_sum = 0.0, proc_max = 0.0;

  for (const auto &m : imu) {
    double t_imu = sec(m.t_ns);
    if (ci >= cam.size()) break;
    // IMU a little before the first used frame is enough for initialisation.
    if (t_imu < start_s - 5.0) continue;
    ov_core::ImuData d;
    d.timestamp = t_imu;
    d.wm << m.w[0], m.w[1], m.w[2];
    d.am << m.a[0], m.a[1], m.a[2];
    sys->feed_measurement_imu(d);

    double dt_cam_imu = sys->get_state()->_calib_dt_CAMtoIMU->value()(0);
    while (ci < cam.size() && sec(cam[ci].t_ns) < t_imu - dt_cam_imu) {
      double t_cam = sec(cam[ci].t_ns);
      if (end_s > 0 && t_cam > end_s) { ci = cam.size(); break; }
      cv::Mat img = cv::imread(root + "/mav0/cam0/data/" + cam[ci].file, cv::IMREAD_GRAYSCALE);
      if (img.empty()) { n_missing++; ci++; continue; }
      if (scale != 1.0) cv::resize(img, img, cv::Size(), scale, scale, cv::INTER_AREA);
      ov_core::CameraData c;
      c.timestamp = t_cam;
      c.sensor_ids.push_back(0);
      c.images.push_back(img);
      c.masks.push_back(cv::Mat::zeros(img.rows, img.cols, CV_8UC1));
      auto w0 = std::chrono::steady_clock::now();
      sys->feed_measurement_camera(c);
      double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - w0).count();
      proc_sum += ms;
      proc_max = std::max(proc_max, ms);
      n_frames++;
      ci++;

      if (!sys->initialized()) continue;
      auto state = sys->get_state();
      double t_off = state->_calib_dt_CAMtoIMU->value()(0);
      double t_state = state->_timestamp + t_off;
      Eigen::Vector4d q = state->_imu->quat();   // JPL q_GtoI == Hamilton q_ItoG, order x y z w
      Eigen::Vector3d p = state->_imu->pos(), v = state->_imu->vel();
      std::vector<std::shared_ptr<ov_type::Type>> vars = {state->_imu->pose()};
      Eigen::MatrixXd P = ov_msckf::StateHelper::get_marginal_covariance(state, vars);  // [ori(3) pos(3)]
      out << t_state << ',' << (t0_ns + static_cast<int64_t>(std::llround(t_state * 1e9))) << ','
          << p(0) << ',' << p(1) << ',' << p(2) << ','
          << q(0) << ',' << q(1) << ',' << q(2) << ',' << q(3) << ','
          << v(0) << ',' << v(1) << ',' << v(2) << ','
          << P(3, 3) << ',' << P(3, 4) << ',' << P(3, 5) << ',' << P(4, 4) << ',' << P(4, 5) << ',' << P(5, 5) << ','
          << P(0, 0) << ',' << P(1, 1) << ',' << P(2, 2) << ','
          << t_off << ',' << sys->get_features_SLAM().size() << ',' << sys->get_good_features_MSCKF().size() << ','
          << ms << '\n';
      n_states++;
    }
  }
  out.close();

  std::ofstream sum(out_dir + "/openvins_run.json");
  sum.precision(12);
  sum << "{\n"
      << "  \"config\": \"" << config << "\",\n"
      << "  \"dataset_root\": \"" << root << "\",\n"
      << "  \"t0_ns\": " << t0_ns << ",\n"
      << "  \"start_s\": " << start_s << ",\n  \"end_s\": " << end_s << ",\n  \"scale\": " << scale << ",\n"
      << "  \"imu_rows\": " << imu.size() << ",\n  \"camera_rows\": " << cam.size() << ",\n"
      << "  \"frames_fed\": " << n_frames << ",\n  \"frames_missing_on_disk\": " << n_missing << ",\n"
      << "  \"states_written\": " << n_states << ",\n"
      << "  \"initialized\": " << (sys->initialized() ? "true" : "false") << ",\n"
      << "  \"proc_ms_mean\": " << (n_frames ? proc_sum / n_frames : 0.0) << ",\n"
      << "  \"proc_ms_max\": " << proc_max << ",\n"
      << "  \"final_dt_cam_imu\": " << sys->get_state()->_calib_dt_CAMtoIMU->value()(0) << "\n}\n";
  std::printf("[run_asl_folder] frames fed %zu, states written %zu, initialized %d\n", n_frames, n_states,
              sys->initialized() ? 1 : 0);
  return n_states > 0 ? 0 : 1;
}
