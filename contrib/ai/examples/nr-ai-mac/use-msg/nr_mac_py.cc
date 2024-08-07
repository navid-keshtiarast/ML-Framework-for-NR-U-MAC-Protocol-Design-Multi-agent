#include "nr-mac-env.h"

#include <ns3/ai-module.h>

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(ns3ai_nrmac_msg_py, m)
{
    py::class_<ns3::NrMacEnv>(m, "PyEnvStruct")
        .def(py::init<>())
        .def_readwrite("envType", &ns3::NrMacEnv::envType)
        .def_readwrite("simTime", &ns3::NrMacEnv::simTime)
        .def_readwrite("minCw1", &ns3::NrMacEnv::minCw1)
        .def_readwrite("minCw2", &ns3::NrMacEnv::minCw2)
        .def_readwrite("mcot1", &ns3::NrMacEnv::mcot1)
        .def_readwrite("mcot2", &ns3::NrMacEnv::mcot2)
        .def_readwrite("trafficType1", &ns3::NrMacEnv::trafficType1)
        .def_readwrite("trafficType2", &ns3::NrMacEnv::trafficType2)
        .def_readwrite("packetSize1", &ns3::NrMacEnv::packetSize1)
        .def_readwrite("packetSize2", &ns3::NrMacEnv::packetSize2)
        .def_readwrite("udpLambda1", &ns3::NrMacEnv::udpLambda1)
        .def_readwrite("udpLambda2", &ns3::NrMacEnv::udpLambda2)
        .def_readwrite("throughput1", &ns3::NrMacEnv::throughput1)
        .def_readwrite("throughput2", &ns3::NrMacEnv::throughput2);

    py::class_<ns3::NrMacAct>(m, "PyActStruct")
        .def(py::init<>())
        .def_readwrite("new_minCw1", &ns3::NrMacAct::new_minCw1)
        .def_readwrite("new_minCw2", &ns3::NrMacAct::new_minCw2)
        .def_readwrite("new_mcot1", &ns3::NrMacAct::new_mcot1)
        .def_readwrite("new_mcot2", &ns3::NrMacAct::new_mcot2)
        .def_readwrite("new_trafficType1", &ns3::NrMacAct::new_trafficType1)
        .def_readwrite("new_trafficType2", &ns3::NrMacAct::new_trafficType2)
        .def_readwrite("new_packetSize1", &ns3::NrMacAct::new_packetSize1)
        .def_readwrite("new_packetSize2", &ns3::NrMacAct::new_packetSize2)
        .def_readwrite("new_udpLambda1", &ns3::NrMacAct::new_udpLambda1)
        .def_readwrite("new_udpLambda2", &ns3::NrMacAct::new_udpLambda2);

    py::class_<ns3::Ns3AiMsgInterfaceImpl<ns3::NrMacEnv, ns3::NrMacAct>>(m, "Ns3AiMsgInterfaceImpl")
        .def(py::init<bool,
                      bool,
                      bool,
                      uint32_t,
                      const char*,
                      const char*,
                      const char*,
                      const char*>())
        .def("PyRecvBegin", &ns3::Ns3AiMsgInterfaceImpl<ns3::NrMacEnv, ns3::NrMacAct>::PyRecvBegin)
        .def("PyRecvEnd", &ns3::Ns3AiMsgInterfaceImpl<ns3::NrMacEnv, ns3::NrMacAct>::PyRecvEnd)
        .def("PySendBegin", &ns3::Ns3AiMsgInterfaceImpl<ns3::NrMacEnv, ns3::NrMacAct>::PySendBegin)
        .def("PySendEnd", &ns3::Ns3AiMsgInterfaceImpl<ns3::NrMacEnv, ns3::NrMacAct>::PySendEnd)
        .def("PyGetFinished",
             &ns3::Ns3AiMsgInterfaceImpl<ns3::NrMacEnv, ns3::NrMacAct>::PyGetFinished)
        .def("GetCpp2PyStruct",
             &ns3::Ns3AiMsgInterfaceImpl<ns3::NrMacEnv, ns3::NrMacAct>::GetCpp2PyStruct,
             py::return_value_policy::reference)
        .def("GetPy2CppStruct",
             &ns3::Ns3AiMsgInterfaceImpl<ns3::NrMacEnv, ns3::NrMacAct>::GetPy2CppStruct,
             py::return_value_policy::reference);
}