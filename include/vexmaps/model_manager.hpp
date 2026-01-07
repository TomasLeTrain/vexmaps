#include "vexmaps/localization_model.hpp"
#include <algorithm>
#include <cassert>
#include <mutex>
#include <vector>

namespace vexmaps {

/**
 * @class ManagedModel
 * @brief Simple struct to hold models and related information.
 *
 */
struct ManagedModel {
    vexmaps::LocalizationModel* model;
    std::string name;
    int priority;

    // custom sorting based on priority
    bool operator<(const ManagedModel& rhs) {
        if (this->priority != rhs.priority)
            return this->priority < rhs.priority;
        if (this->name != rhs.name) return this->name < rhs.name;
        return false;
    }

    // used to remove possible duplicate models
    bool operator==(const ManagedModel& rhs) {
        return this->model == rhs.model;
    }
};

/**
 * @class ModelManager
 * @brief Abstracts the management of multiple models into one model, while
 * keeping the same api.
 *
 */
class ModelManager : public LocalizationModel {
  private:
    std::vector<ManagedModel> models;
    vexmaps::LocalizationModel* active_model;
    int init_timeout = 100;
    int task_creation_timeout = 100;

  protected:
    mutable pros::Mutex m_mutex;

  public:
    ModelManager(std::vector<ManagedModel>&& models,
                 vexmaps::LocalizationModel* active_model)
        : models(std::move(models)),
          active_model(active_model) {
        // remove models with nullptr references
        models.erase(std::remove_if(models.begin(),
                                    models.end(),
                                    [](ManagedModel curr) {
                                        return curr.model == nullptr;
                                    }),
                     models.end());

        // sort the models, such that the ones with lower priority get processed
        // first
        std::sort(models.begin(), models.end());

        // remove duplicate models
        auto duplicate_models = std::unique(models.begin(), models.end());
        models.erase(duplicate_models, models.end());
    }

    // initialize all models
    void init() override {
        for (ManagedModel model : models) {
            if (model.model == nullptr) continue;
            model.model->init();
            pros::delay(init_timeout);
        }

        // create each of the tasks
        createTasks();
    }

    /**
     * @brief initializes the tasks for each model
     *
     * @param timeout time between the initialization of the different models -
     * lets models start up before other models start themselves
     */
    void createTasks() {
        for (ManagedModel model : models) {
            pros::Task model_task(
              [model] {
                  // the models should be global, so they should outlive the
                  // program?
                  while (model.model != nullptr) {
                      uint32_t current_time = pros::millis();
                      model.model->update();

                      pros::c::task_delay_until(
                        &current_time,
                        to_msec(model.model->getTaskDeltaTime()));
                  }
                  std::cout << "stopped task for model " << model.name
                            << std::endl;
              },
              model.name.c_str());
            pros::delay(task_creation_timeout);
        }
    }

    /**
     * @brief Changes the active model. If not a valid model then the model does
     * not get changed.
     *
     * @param new_active_model Pointer of the new model
     */
    void changeActiveModel(vexmaps::LocalizationModel* new_active_model) {
        if (new_active_model == nullptr) return;

        // only change active model if its one we are keeping track of
        for (ManagedModel model : models) {
            if (model.model == new_active_model) {
                active_model = new_active_model;
            }
        }
    }

    void update() override {
        assert(false && "Update should never get called for model manager!");
    }

    void setPose(units::Pose new_pose) override {
        assert(active_model != nullptr);
        active_model->setPose(new_pose);
    }

    units::Pose getPose() override {
        assert(active_model != nullptr);
        return active_model->getPose();
    }

    units::Pose getLastPose() override {
        assert(active_model != nullptr);
        return active_model->getLastPose();
    }

    units::Pose getGlobalPoseDelta() override {
        assert(active_model != nullptr);
        return active_model->getGlobalPoseDelta();
    }

    units::Pose getLocalPoseDelta() override {
        assert(active_model != nullptr);
        return active_model->getLocalPoseDelta();
    }

    Length getDistanceTraveled() override {
        assert(active_model != nullptr);
        return active_model->getDistanceTraveled();
    }

    Time getTaskDeltaTime() override {
        assert(active_model != nullptr);
        return active_model->getTaskDeltaTime();
    }

    Time getLatestUpdateTimestamp() override {
        assert(active_model != nullptr);
        return active_model->getLatestUpdateTimestamp();
    }

    // returns a signed distance traveled from the start of tracking
    Length getForwardTravel() override {
        assert(active_model != nullptr);
        return active_model->getForwardTravel();
    }

    // returns local velocity vector relative to the robot
    units::V2Velocity getLocalVelocityVector() override {
        assert(active_model != nullptr);
        return active_model->getLocalVelocityVector();
    }

    // returns the latest angular velocity
    AngularVelocity getAngularVelocity() override {
        assert(active_model != nullptr);
        return active_model->getAngularVelocity();
    }

    std::optional<Confidences> getConfidence() override {
        assert(active_model != nullptr);
        return active_model->getConfidence();
	}
};
} // namespace vexmaps
