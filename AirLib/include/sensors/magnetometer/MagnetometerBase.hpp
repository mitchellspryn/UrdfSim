// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_MagnetometerBase_hpp
#define msr_airlib_MagnetometerBase_hpp


#include "sensors/SensorBase.hpp"


namespace msr { namespace airlib {

class MagnetometerBase  : public SensorBase {
public:
    MagnetometerBase(const std::string& sensor_name = "", const std::string& attach_link_name = "")
        : SensorBase(sensor_name, attach_link_name)
    {}

public: //types
    struct Output { //same fields as ROS message
        Vector3r magnetic_field_body; //in Gauss
        vector<real_T> magnetic_field_covariance; //9 elements 3x3 matrix    
    };


public:
    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        UpdatableObject::reportState(reporter);

        reporter.writeValue("Mag-Vec", output_.magnetic_field_body);
    }

    virtual const std::map<std::string, float> read() const override
    {
        std::map<std::string, float> values;
        auto& output = this->getOutput();

        values["Mag-Vec-x"] = output.magnetic_field_body.x();
        values["Mag-Vec-y"] = output.magnetic_field_body.y();
        values["Mag-Vec-z"] = output.magnetic_field_body.z();
        values["Mag-Vec-Cov-x-x"] = output.magnetic_field_covariance[0];
        values["Mag-Vec-Cov-x-y"] = output.magnetic_field_covariance[1];
        values["Mag-Vec-Cov-x-z"] = output.magnetic_field_covariance[2];
        values["Mag-Vec-Cov-y-y"] = output.magnetic_field_covariance[4];
        values["Mag-Vec-Cov-y-z"] = output.magnetic_field_covariance[5];
        values["Mag-Vec-Cov-z-z"] = output.magnetic_field_covariance[8];

        return values;
    }

    const Output& getOutput() const
    {
        return output_;
    }

protected:
    void setOutput(const Output& output)
    {
        output_ = output;
    }

private:
    Output output_;
};


}} //namespace
#endif 
