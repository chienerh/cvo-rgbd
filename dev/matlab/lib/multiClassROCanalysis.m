function output = multiClassROCanalysis(y, gt)
% Multi-class ROC analysis; the output is the average of individual AUC
% Inputs:
% y: matrix of probabilistic labels; rows are observations and columns are
% classes.
% gt: ground truth labels
% 
% output: individual ROC analysis results + average AUC of all classes

[~, n] = size(y);
valid_class_number = 0;

roc = cell(n,1);
AUCtotal = 0; sensitivityTotal = 0; accuracyTotal = 0; F1Total = 0;
precisionTotal = 0;

for i = 1:n
    % Set the current class label as 1 and the rest as 0
    idx = gt == i;
    gtLabel = gt; gtLabel(idx) = 1; gtLabel(~idx) = 0;
    
    if ~sum(gtLabel) || any(isnan(y(:,i)))  % class i does not exsit in gt
        roc{i}.isnan = true;
        continue
    end
   
    % AUC for class i
    roc{i} = ROCanalysis(y(:,i), gtLabel);
    roc{i}.isnan = false;
    
    AUCtotal = AUCtotal  + roc{i}.auc;
    sensitivityTotal = sensitivityTotal  + mean(roc{i}.sensitivity(2:end-1));
    accuracyTotal = accuracyTotal  + mean(roc{i}.accuracy(2:end-1));
    F1Total = F1Total  + mean(roc{i}.f1_score(2:end-1));
    precisionTotal = precisionTotal  + mean(roc{i}.precision(2:end-1));
    valid_class_number = valid_class_number + 1;
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
output.roc = roc;