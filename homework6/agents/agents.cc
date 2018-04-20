// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework6/agents/agents.h"

#include "homework6/agents/sample/sample_agent.h"
#include "homework6/agents/tlau/agent.h"
#include "homework6/agents/tlau/straight.h"
#include "homework6/agents/tlau/curve.h"

// Register sample vehicle agent to a factory with its type name "sample_agent"
static simulation::Registrar<::sample::SampleVehicleAgent> registrar("sample_agent");
static simulation::Registrar<::tlau::SampleVehicleAgent> registrar_tlau("tlau_agent");
static simulation::Registrar<::tlau::LineAgent> registrar_tlau_straight("tlau_straight_agent");
static simulation::Registrar<::tlau::CurveAgent> registrar_tlau_curve("tlau_curve_agent");

