function open_sysid_app(trainingCsv, validationCsv, axisName)
%OPEN_SYSID_APP Prepare iddata variables and open System Identification.

if nargin < 3, axisName = "yaw"; end
trainingTable = localRead(trainingCsv);
validationTable = localRead(validationCsv);
Ts = median(diff(double(trainingTable.timestamp_ms))) / 1000.0;
if string(axisName) == "pitch"
    outputSignal = "gyro_y_rad_s";
    outputName = "Pitch angular rate";
    outputUnit = "rad/s";
else
    outputSignal = "speed_rpm";
    outputName = "Motor speed";
    outputUnit = "rpm";
end
trainingOutput = double(trainingTable.(outputSignal));
validationOutput = double(validationTable.(outputSignal));
trainingData = iddata(trainingOutput - mean(trainingOutput), ...
                      double(trainingTable.output_current), Ts, ...
                      "OutputName", outputName, "OutputUnit", outputUnit, ...
                      "InputName", "Current command", "InputUnit", "command");
validationData = iddata(validationOutput - mean(validationOutput), ...
                        double(validationTable.output_current), Ts, ...
                        "OutputName", outputName, "OutputUnit", outputUnit, ...
                        "InputName", "Current command", "InputUnit", "command");
assignin("base", "sysid_training_data", trainingData);
assignin("base", "sysid_validation_data", validationData);
systemIdentification;
end

function data = localRead(path)
data = sortrows(readtable(path), "sample_index");
data.output_current = data.output_current - mean(data.output_current);
end
