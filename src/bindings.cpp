#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "humanorid_sdk.h"
#include "session.hpp"
namespace py = pybind11;
using namespace lpf;
void bind_kinematics(py::module_&);
namespace {
template <class A>
void finite_math(const A& a) {
  for (double x : a)
    if (!std::isfinite(x)) throw std::invalid_argument("mathematical inputs must be finite");
}
std::vector<double> v(const Q7& a) { return {a.begin(), a.end()}; }
std::vector<double> v6(const std::array<double, 6>& a) { return {a.begin(), a.end()}; }
py::str decoded(const std::string& s) {
  return py::bytes(s).attr("decode")("utf-8", "replace").cast<py::str>();
}
struct MathResult {
  bool output_finite = false;
  int code = 1;
  Pose transform{};
  double arm_angle_deg = std::numeric_limits<double>::quiet_NaN();
  Q7 q_deg{};
  std::array<std::array<double, 7>, 16> all_solutions_deg{};
};
void check_dh(const DHParam& d, const DHCMPSTPARAM& c) {
  finite_math(d.DH_a);
  finite_math(d.DH_d);
  finite_math(d.DH_alpha);
  finite_math(d.DH_beta);
  finite_math(d.DH_theta);
  finite_math(c.a);
  finite_math(c.d);
  finite_math(c.alpha);
  finite_math(c.beta);
  finite_math(c.theta);
}
}  // namespace
PYBIND11_MODULE(_native, m) {
  m.doc() = "C++ dual-arm isolation, real SDK math, and vendored fairino_ik";
  bind_kinematics(m);
  py::enum_<SingleRobot::RobotType> type(m, "SDKRobotType");
  type.value("ART3_R7", SingleRobot::ART3_R7);
  type.value("ART3_R7_JK_V1_0", SingleRobot::ART3_R7_JK_V1_0);
  type.value("ART3_R7_JK_V2_0", SingleRobot::ART3_R7_JK_V2_0);
  type.value("ART3_R7_BY", SingleRobot::ART3_R7_BY);
  py::enum_<SingleRobot::RobotName>(m, "SDKRobotName")
      .value("SingleArm", SingleRobot::SingleArm)
      .value("LeftArm", SingleRobot::LeftArm)
      .value("RightArm", SingleRobot::RightArm);
  py::enum_<ARMErrorCode> ec(m, "ARMErrorCode");
  ec.value("Success", ARMErrorCode::Success);
  ec.value("InvalidInput_Nullptr", ARMErrorCode::InvalidInput_Nullptr);
  ec.value("InvalidInput_OutOfRange", ARMErrorCode::InvalidInput_OutOfRange);
  ec.value("InvalidOutput_Nullptr", ARMErrorCode::InvalidOutput_Nullptr);
  ec.value("TCP_ConnectErr", ARMErrorCode::TCP_ConnectErr);
  ec.value("Kine_LibLoadErr", ARMErrorCode::Kine_LibLoadErr);
  ec.value("Kine_KineInitErr", ARMErrorCode::Kine_KineInitErr);
  ec.value("Kine_SingularErr", ARMErrorCode::Kine_SingularErr);
  ec.value("Kine_NumberErr", ARMErrorCode::Kine_NumberErr);
  ec.value("Kine_PositionDeviationErr", ARMErrorCode::Kine_PositionDeviationErr);
  ec.value("Kine_PostureDeviationErr", ARMErrorCode::Kine_PostureDeviationErr);
  ec.value("Traj_TimePlanErr", ARMErrorCode::Traj_TimePlanErr);
  ec.value("Traj_JointLimitsErr", ARMErrorCode::Traj_JointLimitsErr);
  ec.value("Traj_InterpErr", ARMErrorCode::Traj_InterpErr);
  ec.value("Traj_ArcCalcErr", ARMErrorCode::Traj_ArcCalcErr);
  ec.value("TCPError_NotSocketCreate", ARMErrorCode::TCPError_NotSocketCreate);
  ec.value("TCPError_NotSocketConnect", ARMErrorCode::TCPError_NotSocketConnect);
  ec.value("TCPError_NotKeepAlive", ARMErrorCode::TCPError_NotKeepAlive);
  ec.value("TCPError_NotNetwork", ARMErrorCode::TCPError_NotNetwork);
  ec.value("TCPError_DateError", ARMErrorCode::TCPError_DateError);
  ec.value("CMD_RunErr", ARMErrorCode::CMD_RunErr);
  ec.value("DefaultErr", ARMErrorCode::DefaultErr);

  py::class_<HardwareConfig>(m, "HardwareConfig")
      .def(py::init([](SingleRobot::RobotType type, Q14 lower, Q14 upper,
                       std::array<int, 14> permutation, Q14 signs, Q14 offsets, Q14 velocity,
                       std::string model_evidence, std::string mapping_evidence,
                       std::array<double, 28> left_comp, std::array<double, 28> right_comp,
                       int timeout, int watchdog, std::optional<std::array<double, 5>> fixed_torso,
                       std::optional<double> fixed_head, std::string frame_evidence) {
             HardwareConfig c{static_cast<int>(type),
                              lower,
                              upper,
                              signs,
                              offsets,
                              velocity,
                              permutation,
                              std::move(model_evidence),
                              std::move(mapping_evidence),
                              left_comp,
                              right_comp,
                              timeout,
                              watchdog};
             c.fixed_torso_rad = fixed_torso;
             c.fixed_head_rad = fixed_head;
             c.cartesian_frame_evidence = std::move(frame_evidence);
             c.validate();
             return c;
           }),
           py::arg("sdk_robot_type"), py::arg("lower_sdk_deg"), py::arg("upper_sdk_deg"),
           py::arg("sdk_to_model_permutation"), py::arg("sdk_signs"), py::arg("sdk_offsets_deg"),
           py::arg("max_model_velocity_rad_s"), py::arg("model_compatibility_evidence"),
           py::arg("joint_mapping_evidence"),
           py::arg("left_dh_compensation") = std::array<double, 28>{},
           py::arg("right_dh_compensation") = std::array<double, 28>{},
           py::arg("rpc_timeout_ms") = 2000, py::arg("worker_watchdog_ms") = 500,
           py::arg("fixed_torso_rad") = py::none(), py::arg("fixed_head_rad") = py::none(),
           py::arg("cartesian_frame_evidence") = "")
      .def_readonly("sdk_robot_type", &HardwareConfig::sdk_robot_type)
      .def_readonly("model_compatibility_evidence", &HardwareConfig::model_compatibility_evidence)
      .def_readonly("joint_mapping_evidence", &HardwareConfig::joint_mapping_evidence);
  py::class_<SdkReply>(m, "SdkReply")
      .def_readonly("code", &SdkReply::code)
      .def_property_readonly("error", [](const SdkReply& r) { return decoded(r.error); })
      .def_property_readonly("error_bytes", [](const SdkReply& r) { return py::bytes(r.error); })
      .def_readonly("values", &SdkReply::values)
      .def_readonly("arm_angle_deg", &SdkReply::angle)
      .def_readonly("state", &SdkReply::state)
      .def_property_readonly("ok", &SdkReply::ok);
  py::class_<Snapshot>(m, "Snapshot")
      .def_readonly("q_rad", &Snapshot::q_rad)
      .def_readonly("arms", &Snapshot::arms)
      .def_readonly("host_read_begin_ns", &Snapshot::host_read_begin_ns)
      .def_readonly("host_read_end_ns", &Snapshot::host_read_end_ns)
      .def_readonly("valid", &Snapshot::valid)
      .def_readonly("hardware_freshness_known", &Snapshot::hardware_freshness_known)
      .def_property_readonly("hardware_timestamp_ns", [](const Snapshot&) { return py::none(); });
  py::class_<Session, std::shared_ptr<Session>>(m, "Session")
      .def(py::init<HardwareConfig, std::string, std::string>(), py::arg("config"),
           py::arg("worker_executable") = "", py::arg("backend_override") = "")
      .def("connect", &Session::connect, py::arg("deny_network") = false,
           py::call_guard<py::gil_scoped_release>())
      .def("close", &Session::close, py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("connected", &Session::connected)
      .def_property_readonly("diagnostic", &Session::diagnostic)
      .def_property_readonly("log_directory", &Session::log_directory)
      .def_property_readonly("last_worker_pid", &Session::last_worker_pid)
      .def_property_readonly("left", [](std::shared_ptr<Session> s) { return s->arm(0); })
      .def_property_readonly("right", [](std::shared_ptr<Session> s) { return s->arm(1); })
      .def("servo_joints", &Session::servo_joints, py::arg("model_q_rad"), py::arg("period_ms"),
           py::call_guard<py::gil_scoped_release>())
      .def("read_state", &Session::read_state, py::call_guard<py::gil_scoped_release>())
      .def("sdk_degrees", &Session::sdk_degrees, py::arg("model_q_rad"))
      .def(
          "__enter__",
          [](std::shared_ptr<Session> s) {
            if (!s->connected())
              throw std::runtime_error("call connect explicitly before entering a session context");
            return s;
          })
      .def("__exit__", [](Session& s, py::object, py::object, py::object) {
        py::gil_scoped_release release;
        s.close();
      });
  py::class_<Arm> arm(m, "Arm");
  arm.def(
      "MoveJ",
      [](Arm& a, Q7 q, double speed, double acceleration) {
        return a.session->call(a.side, Op::MoveJ, v(q), 0, speed, acceleration);
      },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "MoveP",
      [](Arm& a, std::array<double, 6> pose, double angle, double speed, double acceleration) {
        return a.session->call(a.side, Op::MoveP, v6(pose), 0, speed, acceleration, angle);
      },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "MoveL",
      [](Arm& a, std::array<double, 6> pose, double angle, double speed, double acceleration) {
        return a.session->call(a.side, Op::MoveL, v6(pose), 0, speed, acceleration, angle);
      },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "MoveC",
      [](Arm& a, std::array<double, 6> mid, double mid_angle, std::array<double, 6> end,
         double end_angle, double speed, double acceleration) {
        auto data = v6(mid);
        data.push_back(mid_angle);
        data.insert(data.end(), end.begin(), end.end());
        return a.session->call(a.side, Op::MoveC, data, 0, speed, acceleration, end_angle);
      },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "ServoCart",
      [](Arm& a, const std::vector<Q7>& points) {
        std::vector<double> data;
        for (const auto& p : points) data.insert(data.end(), p.begin(), p.end());
        return a.session->call(a.side, Op::ServoCart, data, points.size());
      },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "ServoJ",
      [](Arm& a, Q7 q, int period_ms) {
        return a.session->call(a.side, Op::ServoJ, v(q), period_ms);
      },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "SetToolCorrd",
      [](Arm& a, int tool_id, std::array<double, 6> pose) {
        return a.session->call(a.side, Op::SetToolCorrd, v6(pose), tool_id);
      },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "GetForwardKin",
      [](Arm& a, Q7 q) { return a.session->call(a.side, Op::GetForwardKin, v(q)); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "GetInverseKin",
      [](Arm& a, std::array<double, 6> pose, double angle) {
        return a.session->call(a.side, Op::GetInverseKin, v6(pose), 0, 0, 0, angle);
      },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "GetInverseKinRef",
      [](Arm& a, std::array<double, 6> pose, double angle, Q7 ref) {
        auto data = v6(pose);
        data.insert(data.end(), ref.begin(), ref.end());
        return a.session->call(a.side, Op::GetInverseKinRef, data, 0, 0, 0, angle);
      },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "GetActualToolFlangePose",
      [](Arm& a, Q7 q) { return a.session->call(a.side, Op::GetActualToolFlangePose, v(q)); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "SetAccScale",
      [](Arm& a, double percentage) {
        return a.session->call(a.side, Op::SetAccScale, {}, 0, percentage);
      },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "SetSpeed",
      [](Arm& a, double percentage) {
        return a.session->call(a.side, Op::SetSpeed, {}, 0, percentage);
      },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "AxisEnable",
      [](Arm& a, int joint_id) { return a.session->call(a.side, Op::AxisEnable, {}, joint_id); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "AxisDisable",
      [](Arm& a, int joint_id) { return a.session->call(a.side, Op::AxisDisable, {}, joint_id); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "AxisZeroing",
      [](Arm& a, int joint_id) { return a.session->call(a.side, Op::AxisZeroing, {}, joint_id); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "SetLimitPositive",
      [](Arm& a, Q7 limits) { return a.session->call(a.side, Op::SetLimitPositive, v(limits)); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "SetLimitNegative",
      [](Arm& a, Q7 limits) { return a.session->call(a.side, Op::SetLimitNegative, v(limits)); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "ServoMoveStart", [](Arm& a) { return a.session->call(a.side, Op::ServoMoveStart); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "ServoMoveEnd", [](Arm& a) { return a.session->call(a.side, Op::ServoMoveEnd); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "StopMotion", [](Arm& a) { return a.session->call(a.side, Op::StopMotion); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "GetActualJointPosDegree",
      [](Arm& a) { return a.session->call(a.side, Op::GetActualJointPosDegree); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "GetRobotMotionDone", [](Arm& a) { return a.session->call(a.side, Op::GetRobotMotionDone); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "GetEEVel", [](Arm& a) { return a.session->call(a.side, Op::GetEEVel); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "AxisResetError", [](Arm& a) { return a.session->call(a.side, Op::AxisResetError); },
      py::call_guard<py::gil_scoped_release>());
  arm.def(
      "GetJointSoftLimitDeg",
      [](Arm& a) { return a.session->call(a.side, Op::GetJointSoftLimitDeg); },
      py::call_guard<py::gil_scoped_release>());

  py::class_<ControllerStatus>(m, "ControllerStatus")
      .def_readonly("running", &ControllerStatus::running)
      .def_readonly("target_sequence", &ControllerStatus::target_sequence)
      .def_readonly("sent_sequence", &ControllerStatus::sent_sequence)
      .def_readonly("host_send_end_ns", &ControllerStatus::host_send_end_ns)
      .def_readonly("commanded_q_rad", &ControllerStatus::commanded_q_rad)
      .def_property_readonly("error", [](const ControllerStatus& s) { return decoded(s.error); });
  py::class_<Controller>(m, "Controller")
      .def(py::init<std::shared_ptr<Session>, Q14, int, int>(), py::arg("session"),
           py::arg("initial_model_q_rad"), py::arg("period_ms") = 5,
           py::arg("command_timeout_ms") = 500)
      .def("start", &Controller::start, py::call_guard<py::gil_scoped_release>())
      .def("stop", &Controller::stop, py::call_guard<py::gil_scoped_release>())
      .def("submit_joints", &Controller::submit_joints, py::arg("model_q_rad"),
           py::call_guard<py::gil_scoped_release>())
      .def("submit_cartesian", &Controller::submit_cartesian, py::arg("model"),
           py::arg("left_pose"), py::arg("right_pose"), py::arg("q_seed"), py::arg("options"),
           py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("status", &Controller::status);
  auto sdk =
      m.def_submodule("sdk_math", "Official explicit-DH pure mathematics; deg/mm versus DH m/rad");
  auto type_DHParam = py::class_<DHParam>(sdk, "DHParam").def(py::init([] { return DHParam{}; }));
  type_DHParam.def_property(
      "DH_a",
      [](const DHParam& d) {
        Q7 q;
        std::copy(std::begin(d.DH_a), std::end(d.DH_a), q.begin());
        return q;
      },
      [](DHParam& d, Q7 q) {
        finite_math(q);
        std::copy(q.begin(), q.end(), std::begin(d.DH_a));
      });
  type_DHParam.def_property(
      "DH_d",
      [](const DHParam& d) {
        Q7 q;
        std::copy(std::begin(d.DH_d), std::end(d.DH_d), q.begin());
        return q;
      },
      [](DHParam& d, Q7 q) {
        finite_math(q);
        std::copy(q.begin(), q.end(), std::begin(d.DH_d));
      });
  type_DHParam.def_property(
      "DH_alpha",
      [](const DHParam& d) {
        Q7 q;
        std::copy(std::begin(d.DH_alpha), std::end(d.DH_alpha), q.begin());
        return q;
      },
      [](DHParam& d, Q7 q) {
        finite_math(q);
        std::copy(q.begin(), q.end(), std::begin(d.DH_alpha));
      });
  type_DHParam.def_property(
      "DH_beta",
      [](const DHParam& d) {
        Q7 q;
        std::copy(std::begin(d.DH_beta), std::end(d.DH_beta), q.begin());
        return q;
      },
      [](DHParam& d, Q7 q) {
        finite_math(q);
        std::copy(q.begin(), q.end(), std::begin(d.DH_beta));
      });
  type_DHParam.def_property(
      "DH_theta",
      [](const DHParam& d) {
        Q7 q;
        std::copy(std::begin(d.DH_theta), std::end(d.DH_theta), q.begin());
        return q;
      },
      [](DHParam& d, Q7 q) {
        finite_math(q);
        std::copy(q.begin(), q.end(), std::begin(d.DH_theta));
      });
  auto type_DHCMPSTPARAM =
      py::class_<DHCMPSTPARAM>(sdk, "DHCMPSTPARAM").def(py::init([] { return DHCMPSTPARAM{}; }));
  type_DHCMPSTPARAM.def_property(
      "a",
      [](const DHCMPSTPARAM& d) {
        Q7 q;
        std::copy(std::begin(d.a), std::end(d.a), q.begin());
        return q;
      },
      [](DHCMPSTPARAM& d, Q7 q) {
        finite_math(q);
        std::copy(q.begin(), q.end(), std::begin(d.a));
      });
  type_DHCMPSTPARAM.def_property(
      "d",
      [](const DHCMPSTPARAM& d) {
        Q7 q;
        std::copy(std::begin(d.d), std::end(d.d), q.begin());
        return q;
      },
      [](DHCMPSTPARAM& d, Q7 q) {
        finite_math(q);
        std::copy(q.begin(), q.end(), std::begin(d.d));
      });
  type_DHCMPSTPARAM.def_property(
      "alpha",
      [](const DHCMPSTPARAM& d) {
        Q7 q;
        std::copy(std::begin(d.alpha), std::end(d.alpha), q.begin());
        return q;
      },
      [](DHCMPSTPARAM& d, Q7 q) {
        finite_math(q);
        std::copy(q.begin(), q.end(), std::begin(d.alpha));
      });
  type_DHCMPSTPARAM.def_property(
      "beta",
      [](const DHCMPSTPARAM& d) {
        Q7 q;
        std::copy(std::begin(d.beta), std::end(d.beta), q.begin());
        return q;
      },
      [](DHCMPSTPARAM& d, Q7 q) {
        finite_math(q);
        std::copy(q.begin(), q.end(), std::begin(d.beta));
      });
  type_DHCMPSTPARAM.def_property(
      "theta",
      [](const DHCMPSTPARAM& d) {
        Q7 q;
        std::copy(std::begin(d.theta), std::end(d.theta), q.begin());
        return q;
      },
      [](DHCMPSTPARAM& d, Q7 q) {
        finite_math(q);
        std::copy(q.begin(), q.end(), std::begin(d.theta));
      });
  auto limits =
      py::class_<JOINTLIMIT>(sdk, "JOINTLIMIT").def(py::init([] { return JOINTLIMIT{}; }));
  limits.def_property(
      "joint1", [](const JOINTLIMIT& d) { return std::array<double, 2>{d.joint1[0], d.joint1[1]}; },
      [](JOINTLIMIT& d, std::array<double, 2> q) {
        finite_math(q);
        if (q[0] >= q[1]) throw std::invalid_argument("joint lower limit must be below upper");
        std::copy(q.begin(), q.end(), d.joint1);
      });
  limits.def_property(
      "joint2", [](const JOINTLIMIT& d) { return std::array<double, 2>{d.joint2[0], d.joint2[1]}; },
      [](JOINTLIMIT& d, std::array<double, 2> q) {
        finite_math(q);
        if (q[0] >= q[1]) throw std::invalid_argument("joint lower limit must be below upper");
        std::copy(q.begin(), q.end(), d.joint2);
      });
  limits.def_property(
      "joint3", [](const JOINTLIMIT& d) { return std::array<double, 2>{d.joint3[0], d.joint3[1]}; },
      [](JOINTLIMIT& d, std::array<double, 2> q) {
        finite_math(q);
        if (q[0] >= q[1]) throw std::invalid_argument("joint lower limit must be below upper");
        std::copy(q.begin(), q.end(), d.joint3);
      });
  limits.def_property(
      "joint4", [](const JOINTLIMIT& d) { return std::array<double, 2>{d.joint4[0], d.joint4[1]}; },
      [](JOINTLIMIT& d, std::array<double, 2> q) {
        finite_math(q);
        if (q[0] >= q[1]) throw std::invalid_argument("joint lower limit must be below upper");
        std::copy(q.begin(), q.end(), d.joint4);
      });
  limits.def_property(
      "joint5", [](const JOINTLIMIT& d) { return std::array<double, 2>{d.joint5[0], d.joint5[1]}; },
      [](JOINTLIMIT& d, std::array<double, 2> q) {
        finite_math(q);
        if (q[0] >= q[1]) throw std::invalid_argument("joint lower limit must be below upper");
        std::copy(q.begin(), q.end(), d.joint5);
      });
  limits.def_property(
      "joint6", [](const JOINTLIMIT& d) { return std::array<double, 2>{d.joint6[0], d.joint6[1]}; },
      [](JOINTLIMIT& d, std::array<double, 2> q) {
        finite_math(q);
        if (q[0] >= q[1]) throw std::invalid_argument("joint lower limit must be below upper");
        std::copy(q.begin(), q.end(), d.joint6);
      });
  limits.def_property(
      "joint7", [](const JOINTLIMIT& d) { return std::array<double, 2>{d.joint7[0], d.joint7[1]}; },
      [](JOINTLIMIT& d, std::array<double, 2> q) {
        finite_math(q);
        if (q[0] >= q[1]) throw std::invalid_argument("joint lower limit must be below upper");
        std::copy(q.begin(), q.end(), d.joint7);
      });

  py::class_<MathResult>(sdk, "MathResult")
      .def_readonly("code", &MathResult::code)
      .def_readonly("output_finite", &MathResult::output_finite)
      .def_readonly("transform_mm", &MathResult::transform)
      .def_readonly("arm_angle_deg", &MathResult::arm_angle_deg)
      .def_readonly("q_deg", &MathResult::q_deg)
      .def_readonly("all_solutions_deg", &MathResult::all_solutions_deg);
  sdk.def(
      "forwardKinematic",
      [](DHParam dh, DHCMPSTPARAM comp, Q7 q, int joint_index) {
        check_dh(dh, comp);
        finite_math(q);
        if (joint_index != 6 && joint_index != 7)
          throw std::invalid_argument("joint_index must be 6 or 7");
        MathResult result;
        double matrix[4][4];
        for (auto& row : matrix)
          for (double& x : row) x = std::numeric_limits<double>::quiet_NaN();
        result.code =
            forwardKinematic(dh, comp, q.data(), joint_index, matrix, &result.arm_angle_deg);
        for (int i = 0; i < 4; ++i)
          for (int j = 0; j < 4; ++j) result.transform[4 * i + j] = matrix[i][j];
        result.output_finite = std::isfinite(result.arm_angle_deg) &&
                               std::all_of(result.transform.begin(), result.transform.end(),
                                           [](double x) { return std::isfinite(x); });
        return result;
      },
      py::arg("dh"), py::arg("compensation"), py::arg("q_deg"), py::arg("joint_index") = 7,
      py::call_guard<py::gil_scoped_release>());
  sdk.def(
      "inverseKinematic",
      [](DHParam dh, DHCMPSTPARAM comp, JOINTLIMIT limits, int joint_index, double angle,
         Pose matrix, Q7 ref) {
        check_dh(dh, comp);
        finite_math(matrix);
        finite_math(ref);
        if (std::abs(matrix[12]) + std::abs(matrix[13]) + std::abs(matrix[14]) +
                std::abs(matrix[15] - 1) >
            1e-6)
          throw std::invalid_argument("homogeneous matrix last row must be [0,0,0,1]");
        Eigen::Matrix3d rotation;
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j) rotation(i, j) = matrix[4 * i + j];
        if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
            std::abs(rotation.determinant() - 1) > 1e-6)
          throw std::invalid_argument("homogeneous matrix rotation must be SO(3)");
        if (!std::isfinite(angle) || (joint_index != 6 && joint_index != 7))
          throw std::invalid_argument("finite arm angle and joint_index 6 or 7 required");
        std::array<std::array<double, 2>, 7> bounds{{{limits.joint1[0], limits.joint1[1]},
                                                     {limits.joint2[0], limits.joint2[1]},
                                                     {limits.joint3[0], limits.joint3[1]},
                                                     {limits.joint4[0], limits.joint4[1]},
                                                     {limits.joint5[0], limits.joint5[1]},
                                                     {limits.joint6[0], limits.joint6[1]},
                                                     {limits.joint7[0], limits.joint7[1]}}};
        for (const auto& b : bounds)
          if (!std::isfinite(b[0]) || !std::isfinite(b[1]) || b[0] >= b[1])
            throw std::invalid_argument("set all seven JOINTLIMIT bounds first");
        double trans[4][4], all[16][7];
        for (auto& row : all)
          for (double& x : row) x = std::numeric_limits<double>::quiet_NaN();
        for (int i = 0; i < 4; ++i)
          for (int j = 0; j < 4; ++j) trans[i][j] = matrix[4 * i + j];
        MathResult out;
        out.q_deg.fill(std::numeric_limits<double>::quiet_NaN());
        out.code = inverseKinematic(dh, comp, limits, joint_index, angle, trans, ref.data(),
                                    out.q_deg.data(), all);
        for (int i = 0; i < 16; ++i)
          std::copy(std::begin(all[i]), std::end(all[i]), out.all_solutions_deg[i].begin());
        out.output_finite = std::all_of(out.q_deg.begin(), out.q_deg.end(),
                                        [](double x) { return std::isfinite(x); });
        return out;
      },
      py::arg("dh"), py::arg("compensation"), py::arg("limits"), py::arg("joint_index"),
      py::arg("arm_angle_deg"), py::arg("transform_mm"), py::arg("reference_q_deg"),
      py::call_guard<py::gil_scoped_release>());
}
