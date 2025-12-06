#pragma once

#include "localization_model.hpp"
#include "vexmaps/localization_model.hpp"

// wrapper for vexmaps models, compatible with blazing
// implements all tracking motions
namespace vexmaps {
class BlazingWrapper {
    LocalizationModel* model;

  public:
    BlazingWrapper(LocalizationModel* model)
        : model(model) {}

    units::V2Position getPosition() {
        return model->getPose();
    }

    Angle getAngle() {
        return model->getPose().orientation;
    }

    Length getForwardTravel() {
        return model->getForwardTravel();
    }

    Length getDistanceTraveled() {
        return model->getDistanceTraveled();
    }

    LinearVelocity getLinearVelocity() {
        return model->getLocalPoseDelta().x / model->getTaskDeltaTime();
    }

    AngularVelocity getAngularVelocity() {
        return model->getAngularVelocity();
    }
};

} // namespace vexmaps
