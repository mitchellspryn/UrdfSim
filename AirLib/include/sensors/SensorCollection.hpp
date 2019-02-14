// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SensorCollection_hpp
#define msr_airlib_SensorCollection_hpp

#include <unordered_map>
#include <map>
#include "sensors/SensorBase.hpp"
#include "common/UpdatableContainer.hpp"
#include "common/Common.hpp"


namespace msr { namespace airlib {

class SensorCollection : UpdatableObject {
public: //types
    typedef SensorBase* SensorBasePtr;
public:
    void insert(SensorBasePtr sensor, SensorBase::SensorType type)
    {
        auto type_int = static_cast<uint>(type);
        std::string sensor_name = sensor->getName();

        const auto& it = sensors_.find(sensor_name);
        if (it == sensors_.end()) {
            const auto& pair = sensors_.emplace(sensor_name, unique_ptr<SensorBaseContainer>(new SensorBaseContainer()));
            pair.first->second->insert(sensor);
        }
        else {
            it->second->insert(sensor);
        }

        const auto& it_type = sensor_type_mappings_.find(type_int);
        if (it_type == sensor_type_mappings_.end()) {
            const auto& pair = sensor_type_mappings_.emplace(type_int, std::vector<std::string>());
            pair.first->second.emplace_back(sensor_name);
        }
        else {
            it_type->second.emplace_back(sensor_name);
        }
    }

    const SensorBase* getByType(SensorBase::SensorType type, uint index = 0) const
    {
        auto type_int = static_cast<uint>(type);
        auto names_it = sensor_type_mappings_.find(type_int);

        if (names_it == sensor_type_mappings_.end()) {
            return nullptr;
        }

        auto names = names_it->second;
        if (names.size() < index) {
            return nullptr;
        }

        return this->getByName(names[index]);
    }

    const SensorBase* getByName(std::string sensor_name) const
    {
        const auto& it = sensors_.find(sensor_name);
        if (it == sensors_.end()) {
            return nullptr;
        }
        else {
            return it->second->at(0);
        }
    }

    uint size(SensorBase::SensorType type) const
    {
        auto type_int = static_cast<uint>(type);
        const auto& it = sensor_type_mappings_.find(type_int);
        if (it == sensor_type_mappings_.end()) {
            return 0;
        }
        else {
            return static_cast<uint>(it->second.size());
        }
    }

    void initialize(const Kinematics::State* kinematics, const Environment* environment)
    {
        for (auto& pair : sensors_) {
            for (auto& sensor : *pair.second) {
                sensor->initialize(kinematics, environment);
            }
        }
    }

    void initialize(std::function<const msr::airlib::Kinematics::State*(std::string)> state_provider_fxn, const Environment* environment)
    {
        for (auto& pair : sensors_){
            for (auto& sensor : *pair.second) {
                std::string attach_link_name = sensor->getAttachLinkName();
                const msr::airlib::Kinematics::State* kinematics = state_provider_fxn(attach_link_name);
                sensor->initialize(kinematics, environment);
            }
        }
    }

    void clear()
    {
        sensors_.clear();
    }
    
    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        UpdatableObject::reset();

        for (auto& pair : sensors_) {
            pair.second->reset();
        }
    }

    virtual void update() override
    {
        UpdatableObject::update();

        for (auto& pair : sensors_) {
            pair.second->update();
        }
    }

    virtual void reportState(StateReporter& reporter) override
    {
        for (auto& pair : sensors_) {
            pair.second->reportState(reporter);
        }
    }

    //*** End: UpdatableState implementation ***//

    virtual std::map<std::string, std::map<std::string, double> > getReadings() const
    {
        std::map<std::string, std::map<std::string, double> > readings;
        for (auto &kvp : this->sensors_)
        {
            std::map<std::string, double> reading;
            std::string sensor_name = kvp.first;
            for (auto iter = kvp.second.get()->begin(); iter != kvp.second.get()->end(); ++iter)
            {
                auto val = static_cast<SensorBase*>(*iter);
                for (auto& kvp2 : val->read())
                {
                    reading[kvp2.first] = kvp2.second;
                }
            }
            readings[sensor_name] = reading;
        }
        return readings;
    }

private:
    typedef UpdatableContainer<SensorBasePtr> SensorBaseContainer;
    unordered_map<uint, std::vector<std::string>> sensor_type_mappings_;
    unordered_map<std::string, unique_ptr<SensorBaseContainer>> sensors_;
};

}} //namespace
#endif
