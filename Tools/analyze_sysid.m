function result = analyze_sysid(trainingCsv, validationCsv, outputSignal, inputSignal)
%ANALYZE_SYSID Identify current-command -> selected angular-rate response.
% result = analyze_sysid("yaw_train.csv", "yaw_validation.csv")
% The second argument is optional; without it the training record is split 70/30.

arguments
    trainingCsv (1,1) string
    validationCsv (1,1) string = ""
    outputSignal (1,1) string = "speed_rpm"
    inputSignal (1,1) string = "output_current"
end

trainTable = readtable(trainingCsv);
assert(all(trainTable.safety_flags == 0), "Training data contains a safety fault.");
trainTable = cleanSamples(trainTable);

if validationCsv == ""
    splitIndex = max(2, floor(height(trainTable) * 0.70));
    validationTable = trainTable(splitIndex + 1:end, :);
    trainTable = trainTable(1:splitIndex, :);
else
    validationTable = cleanSamples(readtable(validationCsv));
    assert(all(validationTable.safety_flags == 0), "Validation data contains a safety fault.");
end

Ts = median(diff(double(trainTable.timestamp_ms))) / 1000.0;
assert(abs(Ts - 0.001) < 0.0002, "Unexpected sample interval: %.6f s", Ts);

assert(ismember(outputSignal, string(trainTable.Properties.VariableNames)), ...
       "Missing output signal: %s", outputSignal);
assert(ismember(inputSignal, string(trainTable.Properties.VariableNames)), ...
       "Missing input signal: %s", inputSignal);
if outputSignal == "gyro_y_rad_s"
    outputName = "Pitch angular rate";
    outputUnit = "rad/s";
else
    outputName = "Motor speed";
    outputUnit = "rpm";
end
trainOutput = double(trainTable.(outputSignal));
validationOutput = double(validationTable.(outputSignal));
trainOutput = trainOutput - mean(trainOutput);
validationOutput = validationOutput - mean(validationOutput);

trainingData = iddata(trainOutput, ...
                      double(trainTable.(inputSignal)), Ts, ...
                      "OutputName", outputName, "OutputUnit", outputUnit, ...
                      "InputName", "Current command", "InputUnit", "command");
validationData = iddata(validationOutput, ...
                        double(validationTable.(inputSignal)), Ts, ...
                        "OutputName", outputName, "OutputUnit", outputUnit, ...
                        "InputName", "Current command", "InputUnit", "command");

models.firstOrderDelay = procest(trainingData, "P1D");
models.secondOrder = tfest(trainingData, 2, 0);
models.stateSpace2 = ssest(trainingData, 2);

figure("Name", "Plant model validation");
compare(validationData, models.firstOrderDelay, models.secondOrder, models.stateSpace2);
figure("Name", "Identification residuals");
resid(validationData, models.firstOrderDelay, models.secondOrder, models.stateSpace2);

result = struct("sampleTime", Ts, "trainingData", trainingData, ...
                "validationData", validationData, "outputSignal", outputSignal, ...
                "inputSignal", inputSignal, ...
                "models", models);
end

function tableOut = cleanSamples(tableIn)
tableOut = sortrows(tableIn, "sample_index");
[~, uniqueRows] = unique(tableOut.sample_index, "stable");
tableOut = tableOut(uniqueRows, :);
assert(height(tableOut) >= 100, "Not enough valid samples.");
gaps = diff(double(tableOut.sample_index));
assert(all(gaps == 1), "Sample sequence contains dropped records.");
tableOut.output_current = tableOut.output_current - mean(tableOut.output_current);
tableOut.excitation_current = tableOut.excitation_current - mean(tableOut.excitation_current);
end
