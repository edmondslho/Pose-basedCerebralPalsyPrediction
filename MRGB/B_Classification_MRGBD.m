clear;
close all
allMat = {};
allTest = {};

%% Importing 00_25J_MRGBD - Normalised and Rotated data - synthetic dataset

load('00_25J_MRGB/labels.mat');
load('00_25J_MRGB/25J_MRGB_HOJO2D_8bins.mat');
load('00_25J_MRGB/25J_MRGB_HOJO2D_16bins.mat');
load('00_25J_MRGB/25J_MRGB_HOJD2D_8bins.mat');
load('00_25J_MRGB/25J_MRGB_HOJD2D_16bins.mat');
load('00_25J_MRGB/25J_MRGB_FFT_JD_8bins.mat');
load('00_25J_MRGB/25J_MRGB_FFT_JD_16bins.mat');
load('00_25J_MRGB/25J_MRGB_FFT_JO_8bins.mat');
load('00_25J_MRGB/25J_MRGB_FFT_JO_16bins.mat');
load('00_25J_MRGB/25J_MRGB_AngDisp_8bins.mat');
load('00_25J_MRGB/25J_MRGB_AngDisp_16bins.mat');
load('00_25J_MRGB/25J_MRGB_Rel_JO_8bins.mat');
load('00_25J_MRGB/25J_MRGB_Rel_JO_16bins.mat');
load('00_25J_MRGB/25J_MRGB_Rel_JO_AngDis_8bins.mat');
load('00_25J_MRGB/25J_MRGB_Rel_JO_AngDis_16bins.mat');

%% INDIVIDUAL JOINT HISTOGRAMS and Feature Fusion

A8_Ind = [ HOJO2D_LeftWrist8+HOJO2D_LeftElbow8, HOJO2D_LeftKnee8+HOJO2D_LeftAnkle8, HOJO2D_RightWrist8+HOJO2D_RightElbow8, HOJO2D_RightKnee8+HOJO2D_RightAnkle8 ];
A16_Ind = [ HOJO2D_LeftWrist16+HOJO2D_LeftElbow16, HOJO2D_LeftKnee16+HOJO2D_LeftAnkle16, HOJO2D_RightWrist16+HOJO2D_RightElbow16, HOJO2D_RightKnee16+HOJO2D_RightAnkle16 ];

B8_Ind = [ HOJD2D_LeftWrist8 HOJD2D_LeftElbow8 HOJD2D_LeftKnee8 HOJD2D_LeftAnkle8 HOJD2D_RightWrist8 HOJD2D_RightElbow8 HOJD2D_RightKnee8 HOJD2D_RightAnkle8 ];
B16_Ind = [ HOJD2D_LeftWrist16, HOJD2D_LeftElbow16, HOJD2D_LeftKnee16, HOJD2D_LeftAnkle16, HOJD2D_RightWrist16, HOJD2D_RightElbow16, HOJD2D_RightKnee16, HOJD2D_RightAnkle16 ];

C8_Ind = [ FFTJO_LeftWrist8, FFTJO_LeftElbow8, FFTJO_LeftKnee8, FFTJO_LeftAnkle8, FFTJO_RightWrist8, FFTJO_RightElbow8, FFTJO_RightKnee8, FFTJO_RightAnkle8 ];
C16_Ind = [ FFTJO_LeftWrist16, FFTJO_LeftElbow16, FFTJO_LeftKnee16, FFTJO_LeftAnkle16, FFTJO_RightWrist16, FFTJO_RightElbow16, FFTJO_RightKnee16, FFTJO_RightAnkle16 ];

D8_Ind = [ FFTJD_LeftWrist8, FFTJD_LeftElbow8, FFTJD_LeftKnee8, FFTJD_LeftAnkle8, FFTJD_RightWrist8, FFTJD_RightElbow8, FFTJD_RightKnee8, FFTJD_RightAnkle8 ];
D16_Ind = [ FFTJD_LeftWrist16, FFTJD_LeftElbow16, FFTJD_LeftKnee16, FFTJD_LeftAnkle16, FFTJD_RightWrist16, FFTJD_RightElbow16, FFTJD_RightKnee16, FFTJD_RightAnkle16 ];

G8_Ind = [ AngDisp_LeftWrist8, AngDisp_LeftElbow8, AngDisp_LeftKnee8, AngDisp_LeftAnkle8, AngDisp_RightWrist8, AngDisp_RightElbow8, AngDisp_RightKnee8, AngDisp_RightAnkle8 ];
G16_Ind = [ AngDisp_LeftWrist16, AngDisp_LeftElbow16, AngDisp_LeftKnee16, AngDisp_LeftAnkle16, AngDisp_RightWrist16, AngDisp_RightElbow16, AngDisp_RightKnee16, AngDisp_RightAnkle16 ];

H8_Ind = [ HORJO2D_Larm8 HORJO2D_Lleg8 HORJO2D_Rarm8 HORJO2D_Rleg8 ];
H16_Ind = [ HORJO2D_Larm16 HORJO2D_Lleg16 HORJO2D_Rarm16 HORJO2D_Rleg16 ];

I8_Ind = [ HORJAD2D_Larm8 HORJAD2D_Lleg8 HORJAD2D_Rarm8 HORJAD2D_Rleg8 ];
I16_Ind = [ HORJAD2D_Larm16 HORJAD2D_Lleg16 HORJAD2D_Rarm16 HORJAD2D_Rleg16 ];

%% TNSRE Feature sets:

% Individual Features:
TNSRE_Ind = {};
TNSRE_Ind_Lbl = {};
TNSRE_Ind = { A8_Ind, A16_Ind, B8_Ind, B16_Ind, C8_Ind, C16_Ind, D8_Ind, D16_Ind, G8_Ind, G16_Ind, H8_Ind, H16_Ind, I8_Ind, I16_Ind };
TNSRE_Ind_Lbl = {'HOJO2D_8', 'HOJO2D_16', 'HOJD2D_8', 'HOJD2D_16', 'FFTJO_8', 'FFTJO_16', 'FFTJD_8', 'FFTJD_16', 'AngDisp_8', 'AngDisp_16', 'Rel_JO_8', 'Rel_JO_16', 'Rel_JO_AngD_8', 'Rel_JO_AngD_16' };


% Pose Feature Fusion:
TNSRE_PoseFusion = {};
TNSRE_PoseFusion_Lbl = {};
TNSRE_PoseFusion = {[C8_Ind  C16_Ind A8_Ind A16_Ind G8_Ind G16_Ind H8_Ind H16_Ind]};
TNSRE_PoseFusion_Lbl = {'PoseFused'};


% Velocity Feature Fusion:
TNSRE_VelFusion = {};
TNSRE_VelFusion_Lbl = {};
TNSRE_VelFusion =  {[B8_Ind I8_Ind D8_Ind]};
TNSRE_VelFusion_Lbl = {'VelocityFused'};


% Pose-Velocity Fusion:
TNSRE_PoseVel_Fusion = {};
TNSRE_PoseVel_Fusion_Lbl = {};
TNSRE_PoseVel_Fusion = {[C16_Ind I8_Ind A16_Ind B16_Ind G16_Ind D16_Ind H8_Ind]};
TNSRE_PoseVel_Fusion_Lbl = {'PoseVelocity'};

%% Classification

allMat = TNSRE_PoseVel_Fusion;
allTest = TNSRE_PoseVel_Fusion_Lbl;
realIdx =  1:length(labels);
realLbl = labels(realIdx);
numVid = length(realIdx);
A_allRes = zeros(length(allMat), 35);
EnsLbl = {};
NumCycles = 11 ;
LRate = 0.13603;
t = templateTree('MinLeafSize',2,'MaxNumSplits',2);

for j=1:length(allMat)  
    MV8 = allMat{j};
    allAccuracy = 0;
    allNN1Accuracy = 0;
    allNN3Accuracy = 0;
    allLDAAccuracy = 0;
    allENAccuracy = 0;
    allSVM2Accuracy = 0;
    allTreeAccuracy = 0;
    allLogRegAccuracy = 0;
    allSVM2Lbl = zeros(numVid,1);
    allENLbl = zeros(numVid,1);
    allLDALbl = zeros(numVid,1);
    allTreeLbl = zeros(numVid,1);
    allNN3Lbl = zeros(numVid,1);
    allNN1Lbl = zeros(numVid,1);
    allLRLbl = zeros(numVid,1);       
    
    for i=realIdx
        trainMat = MV8;
        trainLbl = labels;

        testMat = trainMat(i,:);
        testLbl = trainLbl(i);

        maxVal = max(trainMat);
        minVal = min(trainMat);
        range = maxVal - minVal;
        range(find(range==0)) = 1;
        
        [Z,mu,sigma] = zscore(trainMat);
         sigma(find(sigma==0)) = 1; 
         
        trainMat(i,:) = []; 
        trainLbl(i) = [];
                
        trainMat = (trainMat-mu)./sigma;
        testMat = (testMat-mu)./sigma;        
         
         % SVM        
                MdlSVM2 = fitcsvm(trainMat,trainLbl);
                res = MdlSVM2.predict(testMat);        
                allSVM2Lbl (i) = res;        
                if res==testLbl
                    allSVM2Accuracy = allSVM2Accuracy + 100;
                end

        % Tree
                MdlTree = fitctree(trainMat, trainLbl);
                res = MdlTree.predict(testMat);
                allTreeLbl(i) = res;
                if res==testLbl
                    allTreeAccuracy = allTreeAccuracy + 100;
                end
 
        % NN1
                MdlNN1 = fitcknn(trainMat,trainLbl,'NumNeighbors',1);
                res = MdlNN1.predict(testMat);
                allNN1Lbl(i) = res;
                if res==testLbl
                    allNN1Accuracy = allNN1Accuracy + 100;
                end
                
        % NN3
                MdlNN3 = fitcknn(trainMat,trainLbl,'NumNeighbors',3);
                res = MdlNN3.predict(testMat);
                allNN3Lbl(i) = res;
                if res==testLbl
                    allNN3Accuracy = allNN3Accuracy + 100;
                end
                
        % LDA
                MdlLinear = fitcdiscr(trainMat, trainLbl,'discrimType','pseudoLinear');
                res = MdlLinear.predict(testMat);
                allLDALbl(i) = res;
                if res==testLbl
                    allLDAAccuracy = allLDAAccuracy + 100;
                end

        % ENSEMBLE
                MdlEnsemble = fitcensemble(trainMat, trainLbl, 'Method', 'LogitBoost', 'ScoreTransform', 'logit', 'Learners',t, 'ScoreTransform', 'logit', 'NumLearningCycles',NumCycles, 'LearnRate',LRate);
                [res, score] = MdlEnsemble.predict(testMat);
                allENLbl(i) = res;
                if res==testLbl
                    allENAccuracy = allENAccuracy + 100;
                end
                EnsLbl{j} = allENLbl;             

        % Logistic Regression      
                C = [0 1;1 0];
                MdlLogReg = fitclinear(trainMat, trainLbl, 'Learner','logistic', 'prior', 'uniform', 'Cost', C);
                [res, score] = MdlLogReg.predict(testMat);
                allLRLbl(i) = res;
                if res==testLbl
                    allLogRegAccuracy = allLogRegAccuracy + 100;
                end

    end
  
%% Collating Clasification Results
    
%SVM        
        all_SVM2_TP = getTP(allSVM2Lbl, realLbl);
        all_SVM2_TN = getTN(allSVM2Lbl, realLbl);
        all_SVM2_FP = getFP(allSVM2Lbl, realLbl);
        all_SVM2_FN = getFN(allSVM2Lbl, realLbl);
        
        SVM2_Sensitivity = (all_SVM2_TP/(all_SVM2_TP + all_SVM2_FN))*100;
        SVM2_Specificity = (all_SVM2_TN/(all_SVM2_TN + all_SVM2_FP))*100;
        SVM2_Precision = (all_SVM2_TP/(all_SVM2_TP + all_SVM2_FP))*100;
        SVM2_Recall = (all_SVM2_TP/(all_SVM2_TP + all_SVM2_FN))*100;
        SVM2_F1score = 2*((SVM2_Precision*SVM2_Recall)/(SVM2_Precision+SVM2_Recall));
        SVM2_MC = ((all_SVM2_TP*all_SVM2_TN)-(all_SVM2_FP*all_SVM2_FN))/sqrt((all_SVM2_TP+all_SVM2_FP)*(all_SVM2_TP+all_SVM2_FN)*(all_SVM2_TN+all_SVM2_FP)*(all_SVM2_TN+all_SVM2_FN))*100;
        
        display([allTest{j} ': The average SVM2 classification accuracy is ' num2str(allSVM2Accuracy/numVid) '%.']);
        display([allTest{j} ': The SVM2 Sensitivity is ' num2str(SVM2_Sensitivity) '%.']);
        display([allTest{j} ': The SVM2 Specificity is ' num2str(SVM2_Specificity) '%.']);
        display([allTest{j} ': The SVM2 F1 Score is ' num2str(SVM2_F1score) '%.']);
        display([allTest{j} ': The SVM2 MC Score is ' num2str(SVM2_MC) '%.']);              
        
%Decision Tree        
            all_Tree_TP = getTP(allTreeLbl, realLbl);
            all_Tree_TN = getTN(allTreeLbl, realLbl);
            all_Tree_FP = getFP(allTreeLbl, realLbl);
            all_Tree_FN = getFN(allTreeLbl, realLbl);
        
            Tree_Sensitivity = (all_Tree_TP/(all_Tree_TP + all_Tree_FN))*100;
            Tree_Specificity = (all_Tree_TN/(all_Tree_TN + all_Tree_FP))*100;
            Tree_Precision = (all_Tree_TP/(all_Tree_TP + all_Tree_FP))*100;
            Tree_Recall = (all_Tree_TP/(all_Tree_TP + all_Tree_FN))*100;
            Tree_F1score = 2*((Tree_Precision*Tree_Recall)/(Tree_Precision+Tree_Recall));
            Tree_MC = ((all_Tree_TP*all_Tree_TN)-(all_Tree_FP*all_Tree_FN))/sqrt((all_Tree_TP+all_Tree_FP)*(all_Tree_TP+all_Tree_FN)*(all_Tree_TN+all_Tree_FP)*(all_Tree_TN+all_Tree_FN))*100;
                
            display([allTest{j} ': The average Tree classification accuracy is ' num2str(allTreeAccuracy/numVid) '%.']);
            display([allTest{j} ': The Tree Sensitivity is ' num2str(Tree_Sensitivity) '%.']);
            display([allTest{j} ': The Tree Specificity is ' num2str(Tree_Specificity) '%.']);
            display([allTest{j} ': The Tree F1 Score is ' num2str(Tree_F1score) '%.']);
            display([allTest{j} ': The Tree MC Score is ' num2str(Tree_MC) '%.']);
        
%NN1        
            all_NN1_TP = getTP(allNN1Lbl, realLbl);
            all_NN1_TN = getTN(allNN1Lbl, realLbl);
            all_NN1_FP = getFP(allNN1Lbl, realLbl);
            all_NN1_FN = getFN(allNN1Lbl, realLbl);
        
            NN1_Sensitivity = (all_NN1_TP/(all_NN1_TP + all_NN1_FN))*100;
            NN1_Specificity = (all_NN1_TN/(all_NN1_TN + all_NN1_FP))*100;
            NN1_Precision = (all_NN1_TP/(all_NN1_TP + all_NN1_FP))*100;
            NN1_Recall = (all_NN1_TP/(all_NN1_TP + all_NN1_FN))*100;
            NN1_F1score = 2*((NN1_Precision*NN1_Recall)/(NN1_Precision+NN1_Recall));
            NN1_MC = ((all_NN1_TP*all_NN1_TN)-(all_NN1_FP*all_NN1_FN))/sqrt((all_NN1_TP+all_NN1_FP)*(all_NN1_TP+all_NN1_FN)*(all_NN1_TN+all_NN1_FP)*(all_NN1_TN+all_NN1_FN))*100;

            display([allTest{j} ': The average NN1 classification accuracy is ' num2str(allNN1Accuracy/numVid) '%.']);
            display([allTest{j} ': The NN1 Sensitivity is ' num2str(NN1_Sensitivity) '%.']);
            display([allTest{j} ': The NN1 Specificity is ' num2str(NN1_Specificity) '%.']);
            display([allTest{j} ': The NN1 F1 Score is ' num2str(NN1_F1score) '%.']);
            display([allTest{j} ': The NN1 MC Score is ' num2str(NN1_MC) '%.']);
                     
%NN3        
            all_NN3_TP = getTP(allNN3Lbl, realLbl);
            all_NN3_TN = getTN(allNN3Lbl, realLbl);
            all_NN3_FP = getFP(allNN3Lbl, realLbl);
            all_NN3_FN = getFN(allNN3Lbl, realLbl);
        
            NN3_Sensitivity = (all_NN3_TP/(all_NN3_TP + all_NN3_FN))*100;
            NN3_Specificity = (all_NN3_TN/(all_NN3_TN + all_NN3_FP))*100;
            NN3_Precision = (all_NN3_TP/(all_NN3_TP + all_NN3_FP))*100;
            NN3_Recall = (all_NN3_TP/(all_NN3_TP + all_NN3_FN))*100;
            NN3_F1score = 2*((NN3_Precision*NN3_Recall)/(NN3_Precision+NN3_Recall));
            NN3_MC = ((all_NN3_TP*all_NN3_TN)-(all_NN3_FP*all_NN3_FN))/sqrt((all_NN3_TP+all_NN3_FP)*(all_NN3_TP+all_NN3_FN)*(all_NN3_TN+all_NN3_FP)*(all_NN3_TN+all_NN3_FN))*100;
        
            display([allTest{j} ': The average NN3 classification accuracy is ' num2str(allNN3Accuracy/numVid) '%.']);
            display([allTest{j} ': The NN3 Sensitivity is ' num2str(NN3_Sensitivity) '%.']);
            display([allTest{j} ': The NN3 Specificity is ' num2str(NN3_Specificity) '%.']);
            display([allTest{j} ': The NN3 F1 Score is ' num2str(NN3_F1score) '%.']);
            display([allTest{j} ': The NN3 MC Score is ' num2str(NN3_MC) '%.']);
        
%LDA        
            all_LDA_TP = getTP(allLDALbl, realLbl);
            all_LDA_TN = getTN(allLDALbl, realLbl);
            all_LDA_FP = getFP(allLDALbl, realLbl);
            all_LDA_FN = getFN(allLDALbl, realLbl);
        
            LDA_Sensitivity = (all_LDA_TP/(all_LDA_TP + all_LDA_FN))*100;
            LDA_Specificity = (all_LDA_TN/(all_LDA_TN + all_LDA_FP))*100;
            LDA_Precision = (all_LDA_TP/(all_LDA_TP + all_LDA_FP))*100;
            LDA_Recall = (all_LDA_TP/(all_LDA_TP + all_LDA_FN))*100;
            LDA_F1score = 2*((LDA_Precision*LDA_Recall)/(LDA_Precision+LDA_Recall));
            LDA_MC = ((all_LDA_TP*all_LDA_TN)-(all_LDA_FP*all_LDA_FN))/sqrt((all_LDA_TP+all_LDA_FP)*(all_LDA_TP+all_LDA_FN)*(all_LDA_TN+all_LDA_FP)*(all_LDA_TN+all_LDA_FN))*100;      
        
            display([allTest{j} ': The average LDA classification accuracy is ' num2str(allLDAAccuracy/numVid) '%.']);
            display([allTest{j} ': The LDA Sensitivity is ' num2str(LDA_Sensitivity) '%.']);
            display([allTest{j} ': The LDA Specificity is ' num2str(LDA_Specificity) '%.']);
            display([allTest{j} ': The LDA F1 Score is ' num2str(LDA_F1score) '%.']);
            display([allTest{j} ': The LDA MC Score is ' num2str(LDA_MC) '%.']);
        
%Ensemble
        
            all_EN_TP = getTP(allENLbl, realLbl);
            all_EN_TN = getTN(allENLbl, realLbl);
            all_EN_FP = getFP(allENLbl, realLbl);
            all_EN_FN = getFN(allENLbl, realLbl);
        
            EN_Sensitivity = (all_EN_TP/(all_EN_TP + all_EN_FN))*100;
            EN_Specificity = (all_EN_TN/(all_EN_TN + all_EN_FP))*100;
            EN_Precision = (all_EN_TP/(all_EN_TP + all_EN_FP))*100;
            EN_Recall = (all_EN_TP/(all_EN_TP + all_EN_FN))*100;
            EN_F1score = 2*((EN_Precision*EN_Recall)/(EN_Precision+EN_Recall));
            EN_MC = ((all_EN_TP*all_EN_TN)-(all_EN_FP*all_EN_FN))/sqrt((all_EN_TP+all_EN_FP)*(all_EN_TP+all_EN_FN)*(all_EN_TN+all_EN_FP)*(all_EN_TN+all_EN_FN))*100;
        
            display([allTest{j} ': The average Ensemble classification accuracy is ' num2str(allENAccuracy/numVid) '%.']);
            display([allTest{j} ': The Ensemble Sensitivity is ' num2str(EN_Sensitivity) '%.']);
            display([allTest{j} ': The Ensemble Specificity is ' num2str(EN_Specificity) '%.']);
            display([allTest{j} ': The Ensemble F1 Score is ' num2str(EN_F1score) '%.']);
            display([allTest{j} ': The Ensemble MC Score is ' num2str(EN_MC) '%.']);

%Logistic Regression        
            all_LR_TP = getTP(allLRLbl, realLbl);
            all_LR_TN = getTN(allLRLbl, realLbl);
            all_LR_FP = getFP(allLRLbl, realLbl);
            all_LR_FN = getFN(allLRLbl, realLbl);
        
            LR_Sensitivity = (all_LR_TP/(all_LR_TP + all_LR_FN))*100;
            LR_Specificity = (all_LR_TN/(all_LR_TN + all_LR_FP))*100;
            LR_Precision = (all_LR_TP/(all_LR_TP + all_LR_FP))*100;
            LR_Recall = (all_LR_TP/(all_LR_TP + all_LR_FN))*100;
            LR_F1score = 2*((LR_Precision*LR_Recall)/(LR_Precision+LR_Recall));
            LR_MC = ((all_LR_TP*all_LR_TN)-(all_LR_FP*all_LR_FN))/sqrt((all_LR_TP+all_LR_FP)*(all_LR_TP+all_LR_FN)*(all_LR_TN+all_LR_FP)*(all_LR_TN+all_LR_FN))*100; 
        
            display([allTest{j} ': The average LogReg classification accuracy is ' num2str(allLogRegAccuracy/numVid) '%.']);
        
            display([allTest{j} ': The LogReg Sensitivity is ' num2str(LR_Sensitivity) '%.']);
            display([allTest{j} ': The LogReg Specificity is ' num2str(LR_Specificity) '%.']);
            display([allTest{j} ': The LogReg F1 Score is ' num2str(LR_F1score) '%.']);
            display([allTest{j} ': The LogReg MC Score is ' num2str(LR_MC) '%.']);
           
%All Classifiers:
             A_allRes(j,:) = [ allSVM2Accuracy/numVid SVM2_Sensitivity SVM2_Specificity SVM2_F1score SVM2_MC allTreeAccuracy/numVid Tree_Sensitivity Tree_Specificity Tree_F1score Tree_MC allNN1Accuracy/numVid NN1_Sensitivity NN1_Specificity NN1_F1score NN1_MC allNN3Accuracy/numVid NN3_Sensitivity NN3_Specificity NN3_F1score NN3_MC allLDAAccuracy/numVid LDA_Sensitivity LDA_Specificity LDA_F1score LDA_MC allENAccuracy/numVid EN_Sensitivity EN_Specificity EN_F1score EN_MC allLogRegAccuracy/numVid LR_Sensitivity LR_Specificity LR_F1score LR_MC];       
         
end

    %% Functions
    
    function accuracy = getAccuracy(predictedLbl, GTLbl)
    accuracy = 0;
    len = length(GTLbl);    
    for i=1:len
        if predictedLbl(i)==GTLbl(i)
            accuracy = accuracy + 1;
        end
    end    
    accuracy = accuracy / len;
    end
    
    
    function TN = getTN(predictedLbl, GTLbl)
    TN = 0;
    len = length(GTLbl);
    TN_Lbl = (GTLbl*2);    
    for i=1:len
        if predictedLbl(i)+ TN_Lbl(i)==0
            TN = TN + 1;
        end        
    end    
    TN = TN;    
    end
    
    function FP = getFP(predictedLbl, GTLbl)
    FP = 0;
    len = length(GTLbl);
    FP_Lbl = (GTLbl*2);    
    for i=1:len
        if predictedLbl(i)+ FP_Lbl(i)==1
            FP = FP + 1;
        end        
    end    
    FP = FP;    
    end
    
    function FN = getFN(predictedLbl, GTLbl)
    FN = 0;
    len = length(GTLbl);
    FN_Lbl = (GTLbl*2);    
    for i=1:len
        if predictedLbl(i)+ FN_Lbl(i)==2
            FN = FN + 1;
        end        
    end    
    FN = FN;    
    end
    
    function TP = getTP(predictedLbl, GTLbl)
    TP = 0;
    len = length(GTLbl);
    TP_Lbl = (GTLbl*2);    
    for i=1:len
        if predictedLbl(i)+ TP_Lbl(i)==3
            TP = TP + 1;
        end        
    end    
    TP = TP;    
    end
