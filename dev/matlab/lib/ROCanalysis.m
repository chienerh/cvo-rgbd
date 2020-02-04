function roc = ROCanalysis(p, gt, varargin)

if ~all(size(gt) == size(p))
    error('The data and ground truth sizes are not compatible.')
end

tcp = find(gt == 1); % total condition positive 
tcn = find(gt == 0); % total condition negative 
[m,n] = size(gt);
len = m*n;

% loop index and increment range
k = 1;
increment = -0.005;
range = 1:increment:0;

% true-positive (TP), true-negative (TN), fals-positive (FP), false-negative (FN) 
tp = 0; tn = 0; fp = 0; fn = 0;
TP = 0; TN = 0; FP = 0; FN = 0;

% metrics
sensitivity = zeros(length(range),1);  % sensitivity, recall, hit rate, or true positive rate (TPR)
specificity = zeros(length(range),1);  % specificity or true negative rate (TNR)
precision = zeros(length(range),1);    % precision or positive predictive value (PPV): TP / (TP  + FP)
accuracy = zeros(length(range),1);     % (TP + TN) / (TP + TN + FP + FN)
f1_score = zeros(length(range),1);     % F1 score: harmonic mean of precision and sensitivity
iou = zeros(length(range),1);          % intersection over union (Jaccard index)
 
for t = range
    for i = 1:len
        if p(i) >= t            % test outcome is positive
            if gt(i) == 1
                tp = tp + 1;
            else
                fp = fp + 1;
            end
%         elseif p(i) < (1-t)            % test outcome is positive
%             if gt(i) == 0
%                 tp = tp + 1;
%             else
%                 fp = fp + 1;
%             end
        else                     % test outcome is negative     
            if gt(i) == 0
                tn = tn + 1;
            else
                fn = fn + 1;
            end
        end
    end
    % evaluate metrics
    sensitivity(k) = tp / (tp + fn);
    specificity(k) = tn / (tn + fp);
    precision(k) = tp / (tp + fp);
    accuracy(k) = (tp + tn) / (tp + tn + fp + fn);
    f1_score(k) = 2 * tp / (2 * tp + fp + fn);
    iou(k) = tp / (tp + fp + fn); 
    
    TP(k) = tp; FP(k) = fp; TN(k) = tn; FN(k) = fn; 
    k = k + 1;
    tp = 0; tn = 0; fp = 0; fn = 0;
end

roc = [];
roc.specificity = specificity;
roc.sensitivity = sensitivity;
roc.precision = precision;
roc.accuracy = accuracy;
roc.f1_score = f1_score;
roc.iou = iou;

roc.tcp = tcp;
roc.tcn = tcn;
roc.TP = TP; roc.FP = FP; roc.TN = TN; roc.FN = FN;
roc.t = range;
roc.increment = increment;
roc.auc = trapz(1-specificity, sensitivity);

if isnan(roc.auc)
    keyboard
end

if nargin > 2
    if varargin{1}
        fsize = 24;
        set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
        figure
        plot(1-specificity, sensitivity, [0,1],[0,1], '--', 'linewidth',2)
        axis([0 1 0 1]), axis tight, grid on
        set(gca,'fontsize',fsize)
        text(.5,.3,['AUC = ' num2str(roc.auc)],'fontsize',fsize, 'Interpreter','latex')
        ylabel('True positive rate','fontsize',fsize, 'Interpreter','latex')
        xlabel('False positive rate','fontsize',fsize, 'Interpreter','latex')
        
        set(gca,'TickLabelInterpreter','latex')
        axis square
        figuresize(21,21,'cm')
    end
end