%% Initialization

clear;
close all;
load('C_25J_MRGB_ProcessedData.mat');
vidRange = 1:12;
numBin = 8;
binInc = 2;

jRightArm = 4:5;
jLeftArm = 7:8;
jRightLeg = 11:12;
jLeftLeg = 14:15;
jFullBody = 1:21;

binVec = zeros(2,numBin);

H_allJoints8 = {};
H_jRightElbow8 = {};
H_jRightWrist8 = {};
H_jLeftElbow8 = {};
H_jLeftWrist8 = {};
H_jRightKnee8 = {};
H_jRightAnkle8 = {};
H_jLeftKnee8 = {};
H_jLeftAnkle8 = {};


%% HOJD2D
for vid = vidRange
    
    for allJoints = jFullBody
        
        frmLength = length(allFinalPose{vid}(1,:))-1;

        curHist = zeros(1,numBin);
        
        for frm = 6:5:frmLength
            for jID = [allJoints]
                
                vel = allFinalPose{vid}{frm}(jID,1:2) -allFinalPose{vid}{frm-5}(jID,1:2);
                speed = norm(vel);
                
                if speed < (1*binInc)
                    bestBin = 1;
                elseif speed < (2*binInc)
                    bestBin = 2;
                elseif speed < (3*binInc)
                    bestBin = 3;
                elseif speed < (4*binInc)
                    bestBin = 4;
                elseif speed < (5*binInc)
                    bestBin = 5;
                elseif speed < (6*binInc)
                    bestBin = 6;
                elseif speed < (7*binInc)
                    bestBin = 7;
                elseif speed >= (7*binInc)
                    bestBin = 8;
                end
                
                curHist(bestBin) = curHist(bestBin) + 1;
            end
        end
        
         normHist = curHist/sum(curHist);        
        
        H_allJoints8{vid}{allJoints}  = normHist;
    end
    
H_jRightElbow8 {vid} = [H_allJoints8{vid}{min(jRightArm)}];
H_jRightWrist8 {vid} = [H_allJoints8{vid}{max(jRightArm)}];

H_jLeftElbow8 {vid} = [H_allJoints8{vid}{min(jLeftArm)}];
H_jLeftWrist8 {vid} = [H_allJoints8{vid}{max(jLeftArm)}];

H_jRightKnee8 {vid} = [H_allJoints8{vid}{min(jRightLeg)}];
H_jRightAnkle8 {vid} = [H_allJoints8{vid}{max(jRightLeg)}];

H_jLeftKnee8 {vid} = [H_allJoints8{vid}{min(jLeftLeg)}];
H_jLeftAnkle8 {vid} = [H_allJoints8{vid}{max(jLeftLeg)}];
        
    
end

%% Reshaping for remaining pipeline

HOJD2D_RightElbow8 = {};
HOJD2D_RightElbow8  = reshape(cell2mat(H_jRightElbow8), (numBin), (numel(vidRange)))';

HOJD2D_RightWrist8 = {};
HOJD2D_RightWrist8  = reshape(cell2mat(H_jRightWrist8), (numBin), (numel(vidRange)))';

HOJD2D_RightKnee8 = {};
HOJD2D_RightKnee8  = reshape(cell2mat(H_jRightKnee8), (numBin), (numel(vidRange)))';

HOJD2D_RightAnkle8 = {};
HOJD2D_RightAnkle8  = reshape(cell2mat(H_jRightAnkle8), (numBin), (numel(vidRange)))';


HOJD2D_LeftElbow8 = {};
HOJD2D_LeftElbow8  = reshape(cell2mat(H_jLeftElbow8), (numBin), (numel(vidRange)))';

HOJD2D_LeftWrist8 = {};
HOJD2D_LeftWrist8  = reshape(cell2mat(H_jLeftWrist8), (numBin), (numel(vidRange)))';

HOJD2D_LeftKnee8 = {};
HOJD2D_LeftKnee8  = reshape(cell2mat(H_jLeftKnee8), (numBin), (numel(vidRange)))';

HOJD2D_LeftAnkle8 = {};
HOJD2D_LeftAnkle8  = reshape(cell2mat(H_jLeftAnkle8), (numBin), (numel(vidRange)))';

% HOJD2D_FullBody8 = {};
% HOJD2D_FullBody8  = reshape(cell2mat(H_jFullBody8), (numBin), (numel(vidRange)))';

%% Export all Histogram data (HOJD2D) for remaining pipeline
save(['25J_MRGB_HOJD2D_' num2str(numBin) 'bins' '.mat'], 'HOJD2D_RightElbow8', 'HOJD2D_RightWrist8', 'HOJD2D_RightKnee8', 'HOJD2D_RightAnkle8', 'HOJD2D_LeftElbow8', 'HOJD2D_LeftWrist8', 'HOJD2D_LeftKnee8', 'HOJD2D_LeftAnkle8');