// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
#ifndef REVOY_H
#define REVOY_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_models/vehicle/kraz/Revoy_Chassis.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// Revoy system.
class CH_MODELS_API Revoy : public ChWheeledVehicle {
public:
  Revoy(CollisionType chassis_collision_type = CollisionType::NONE,
        ChContactMethod contactMethod = ChContactMethod::NSC);
  ~Revoy() {}

  virtual unsigned int GetNumberAxles() const override { return 1; }
  virtual double GetWheelbase() const override { return 4.78; }

  using ChWheeledVehicle::Initialize;
  virtual void Initialize(std::shared_ptr<ChChassis> frontChassis,
                          const ChCoordsys<> &chassisPos,
                          double chassisFwdVel = 0);

  double GetHitchTorque() const {
    const double forceX = m_connector->GetHitchForceX();
    const double wheelRadius = GetWheel(0, LEFT)->GetTire()->GetRadius();
    /// NOTE: the negative on Force is required to make positive torque =
    /// forward accel
    return -forceX * wheelRadius;
  };

private:
  void Create(CollisionType chassis_collision_type);
  std::shared_ptr<Revoy_Chassis> m_chassis_as_rear;
  std::shared_ptr<Revoy_Connector> m_connector;
};

} // end namespace kraz
} // end namespace vehicle
} // end namespace chrono

#endif
