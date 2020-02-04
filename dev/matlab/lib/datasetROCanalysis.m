function output = datasetROCanalysis( roc, class_number)
%DATASETROCANALYSIS Summary of this function goes here
%   Detailed explanation goes here
%% Calculate metrics for the entire dataset

file_number = length(roc);
valid_class_number = 0;

AUCtotal = 0; sensitivityTotal = 0; accuracyTotal = 0; F1Total = 0;
precisionTotal = 0;


increment = -0.005;
range = 1:increment:0;
init_vector = range * 0;

for c = 1 : class_number
    
    % Initialize roctot
    roctot{c}.TP = init_vector;
    roctot{c}.TN = init_vector;
    roctot{c}.FP = init_vector;
    roctot{c}.FN = init_vector;
    roctot{c}.sensitivity = init_vector;
    roctot{c}.specificity = init_vector;
    roctot{c}.accuracy = init_vector;
    roctot{c}.f1_score = init_vector;
    roctot{c}.precision = init_vector;
    roctot{c}.auc = 0;
    
    % Calculate tp tn & fp fn for entire dataset
    for i = 1 : file_number
        try
        if roc{i}.roc{c}.isnan
            continue
        else
            roctot{c}.TP = roctot{c}.TP + roc{i}.roc{c}.TP;
            roctot{c}.TN = roctot{c}.TN + roc{i}.roc{c}.TN;
            roctot{c}.FP = roctot{c}.FP + roc{i}.roc{c}.FP;
            roctot{c}.FN = roctot{c}.FN + roc{i}.roc{c}.FN;
        end
        catch
            keyboard
        end
    end
    
    % Evaluate metrics for c-th class
    tp = roctot{c}.TP;
    tn = roctot{c}.TN;
    fp = roctot{c}.FP;
    fn = roctot{c}.FN;
    roctot{c}.sensitivity = tp ./ (tp + fn);
    roctot{c}.specificity = tn ./ (tn + fp);
    roctot{c}.accuracy = (tp + tn) ./ (tp + tn + fp + fn);
    roctot{c}.f1_score = (2 * tp) ./ (2 * tp + fp + fn);
    roctot{c}.precision = tp ./ (tp + fp);
    roctot{c}.auc = trapz(1 - roctot{c}.specificity, roctot{c}.sensitivity);
    
    % Calculate Average metrics
    if isnan(roctot{c}.auc)
        continue;
    else
        % MGJ: use (2:end-1) to avoid 0 and 1 thresholds
        disp([num2str(c), ' class sensitivity = ', num2str( mean(roctot{c}.sensitivity(2:end-1)) )]);
        disp([' ',        ' class accuracy    = ', num2str( mean(roctot{c}.accuracy(2:end-1)) )]);
        disp([' ',        ' class F1 Score    = ', num2str( mean(roctot{c}.f1_score(2:end-1)) )]);
        disp([' ',        ' class precision   = ', num2str( mean(roctot{c}.precision(2:end-1)) )]);
        disp([' ',        ' class AUC         = ', num2str( roctot{c}.auc )]);
        disp('-------')
        
        AUCtotal = AUCtotal  + roctot{c}.auc;
        sensitivityTotal = sensitivityTotal  + mean(roctot{c}.sensitivity(2:end-1));
        accuracyTotal = accuracyTotal  + mean(roctot{c}.accuracy(2:end-1));
        F1Total = F1Total  + mean(roctot{c}.f1_score(2:end-1));
        precisionTotal = precisionTotal  + mean(roctot{c}.precision(2:end-1));
        valid_class_number = valid_class_number + 1;
    end
end

% Calculate Average metrics
AUCtotal = AUCtotal ./ valid_class_number; % divide by the number of classes
sensitivityTotal = sensitivityTotal ./ valid_class_number;
accuracyTotal = accuracyTotal ./ valid_class_number;
F1Total = F1Total ./ valid_class_number;
precisionTotal = precisionTotal ./ valid_class_number;

output = [];
output.AUCtot = AUCtotal;
output.sensitivityTotal = sensitivityTotal;
output.accuracyTotal = accuracyTotal;
output.F1Total = F1Total;
output.precisionTotal = precisionTotal;
output.roc = roctot;

end

