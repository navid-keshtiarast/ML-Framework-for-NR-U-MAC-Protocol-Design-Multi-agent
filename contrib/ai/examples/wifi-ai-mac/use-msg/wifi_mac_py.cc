#include "wifi-mac-env.h"

#include <ns3/ai-module.h>

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(ns3ai_wifimac_msg_py, m)
{
    py::class_<ns3::WifiMacEnv>(m, "PyEnvStruct")
        .def(py::init<>())
        .def_readwrite("envType", &ns3::WifiMacEnv::envType)
        .def_readwrite("simTime", &ns3::WifiMacEnv::simTime)
        .def_readwrite("minCw1", &ns3::WifiMacEnv::minCw1)
        .def_readwrite("minCw2", &ns3::WifiMacEnv::minCw2)
        .def_readwrite("mcot1", &ns3::WifiMacEnv::mcot1)
        .def_readwrite("mcot2", &ns3::WifiMacEnv::mcot2)
        .def_readwrite("trafficType1", &ns3::WifiMacEnv::trafficType1)
        .def_readwrite("trafficType2", &ns3::WifiMacEnv::trafficType2)
        .def_readwrite("packetSize1", &ns3::WifiMacEnv::packetSize1)
        .def_readwrite("packetSize2", &ns3::WifiMacEnv::packetSize2)
        .def_readwrite("udpLambda1", &ns3::WifiMacEnv::udpLambda1)
        .def_readwrite("udpLambda2", &ns3::WifiMacEnv::udpLambda2)
        .def_readwrite("throughput1", &ns3::WifiMacEnv::throughput1)
        .def_readwrite("throughput2", &ns3::WifiMacEnv::throughput2);

    py::class_<ns3::WifiMacAct>(m, "PyActStruct")
        .def(py::init<>())
        .def_readwrite("new_minCw1", &ns3::WifiMacAct::new_minCw1)
        .def_readwrite("new_minCw2", &ns3::WifiMacAct::new_minCw2)
        .def_readwrite("new_mcot1", &ns3::WifiMacAct::new_mcot1)
        .def_readwrite("new_mcot2", &ns3::WifiMacAct::new_mcot2)
        .def_readwrite("new_trafficType1", &ns3::WifiMacAct::new_trafficType1)
        .def_readwrite("new_trafficType2", &ns3::WifiMacAct::new_trafficType2)
        .def_readwrite("new_packetSize1", &ns3::WifiMacAct::new_packetSize1)
        .def_readwrite("new_packetSize2", &ns3::WifiMacAct::new_packetSize2)
        .def_readwrite("new_udpLambda1", &ns3::WifiMacAct::new_udpLambda1)
        .def_readwrite("new_udpLambda2", &ns3::WifiMacAct::new_udpLambda2);

    py::class_<ns3::Ns3AiMsgInterfaceImpl<ns3::WifiMacEnv, ns3::WifiMacAct>>(m, "Ns3AiMsgInterfaceImpl")
        .def(py::init<bool,
                      bool,
                      bool,
                      uint32_t,
                      const char*,
                      const char*,
                      const char*,
                      const char*>())
        .def("PyRecvBegin", &ns3::Ns3AiMsgInterfaceImpl<ns3::WifiMacEnv, ns3::WifiMacAct>::PyRecvBegin)
        .def("PyRecvEnd", &ns3::Ns3AiMsgInterfaceImpl<ns3::WifiMacEnv, ns3::WifiMacAct>::PyRecvEnd)
        .def("PySendBegin", &ns3::Ns3AiMsgInterfaceImpl<ns3::WifiMacEnv, ns3::WifiMacAct>::PySendBegin)
        .def("PySendEnd", &ns3::Ns3AiMsgInterfaceImpl<ns3::WifiMacEnv, ns3::WifiMacAct>::PySendEnd)
        .def("PyGetFinished",
             &ns3::Ns3AiMsgInterfaceImpl<ns3::WifiMacEnv, ns3::WifiMacAct>::PyGetFinished)
        .def("GetCpp2PyStruct",
             &ns3::Ns3AiMsgInterfaceImpl<ns3::WifiMacEnv, ns3::WifiMacAct>::GetCpp2PyStruct,
             py::return_value_policy::reference)
        .def("GetPy2CppStruct",
             &ns3::Ns3AiMsgInterfaceImpl<ns3::WifiMacEnv, ns3::WifiMacAct>::GetPy2CppStruct,
             py::return_value_policy::reference);
}