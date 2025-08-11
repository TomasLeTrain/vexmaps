#include "vexmaps/localization_model.hpp"
#include <algorithm>
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

    // initializes each model
    void init() override {
        for (auto model : models) {
            printf("processing: %s\n", model.name.c_str());
            model.model->init();
            printf("finished init\n");
            pros::delay(init_timeout);
        }

        printf("creating tasks\n");
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
        for (auto model : models) {
            printf("creating: %s\n", model.name.c_str());
            pros::Task model_task {
                [&] {
                    while (true) {
                        uint32_t current_time = pros::millis();
                        model.model->update();

                        pros::c::task_delay_until(
                          &current_time,
                          to_msec(model.model->getTaskDeltaTime()));
                    }
                },
                model.name.c_str()
            };
            printf("finished making it\n");
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
        for (auto model : models) {
            if (model.model == new_active_model) {
                active_model = new_active_model;
            }
        }
    }

    void update() override {}

    void setPose(units::Pose new_pose) override {
        active_model->setPose(new_pose);
    }

    units::Pose getPose() override {
        return active_model->getPose();
    }

    units::Pose getLastPose() override {
        return active_model->getLastPose();
    }

    units::Pose getGlobalPoseDelta() override {
        return active_model->getGlobalPoseDelta();
    }

    units::Pose getLocalPoseDelta() override {
        return active_model->getLocalPoseDelta();
    }

    std::optional<float> getConfidence() override {
        return active_model->getConfidence();
    }

    Length getDistanceTraveled() override {
        return active_model->getDistanceTraveled();
    }

    Time getTaskDeltaTime() override {
        return active_model->getTaskDeltaTime();
    }

    Time getLatestUpdateTimestamp() override {
        return active_model->getLatestUpdateTimestamp();
    }

    ~ModelManager() override = default;
};
} // namespace vexmaps
