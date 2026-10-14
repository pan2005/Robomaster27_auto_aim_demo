sessionRoot = fileparts(mfilename("fullpath"));
modelRoot = fullfile(sessionRoot, "models");
names = ["yaw_amp1200_sw40ms_models.mat", ...
         "yaw_amp1200_sw20ms_models.mat"];

for k = 1:numel(names)
    loaded = load(fullfile(modelRoot, names(k)));
    result = loaded.result;
    [~, fit] = compare(result.validationData, ...
                       result.models.firstOrderDelay, ...
                       result.models.secondOrder, ...
                       result.models.stateSpace2);
    if iscell(fit)
        fit = cell2mat(fit);
    end
    fprintf("%s FIT firstOrderDelay=%.4f secondOrder=%.4f stateSpace2=%.4f\n", ...
            char(names(k)), fit(1), fit(2), fit(3));
    fprintf("FIRST_ORDER_DELAY\n");
    disp(result.models.firstOrderDelay);
    fprintf("SECOND_ORDER\n");
    disp(result.models.secondOrder);
    fprintf("STATE_SPACE_2\n");
    disp(result.models.stateSpace2);
end
