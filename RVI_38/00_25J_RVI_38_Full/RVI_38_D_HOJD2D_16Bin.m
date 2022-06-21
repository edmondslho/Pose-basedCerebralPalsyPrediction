%% Initialization

clear;
close all;

load('C_25J_RVI_38_ProcessedData.mat');
vidRange = 1:38;
numBin = 16;
binInc = 2; 

jRightArm = 4:5;
jLeftArm = 7:8;
jRightLeg = 11:12;
jLeftLeg = 14:15;
jFullBody = 1:21;

binVec = zeros(2,numBin);

H_allJoints16 = {};
H_jRightElbow16 = {};
H_jRightWrist16 = {};
H_jLeftElbow16 = {};
H_jLeftWrist16 = {};
H_jRightKnee16 = {};
H_jRightAnkle16 = {};
H_jLeftKnee16 = {};
H_jLeftAnkle16 = {};


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
            elseif speed < (8*binInc)
                bestBin = 8;
            elseif speed < (9*binInc)
                bestBin = 9;
            elseif speed < (10*binInc)
                bestBin = 10;
            elseif speed < (11*binInc)
                bestBin = 11;
            elseif speed < (12*binInc)
                bestBin = 12;
            elseif speed < (13*binInc)
                bestBin = 13;
            elseif speed < (14*binInc)
                bestBin = 14;
            elseif speed < (15*binInc)
                bestBin = 15;
            elseif speed >= (15*binInc)
                bestBin = 16;
            end
                
                curHist(bestBin) = curHist(bestBin) + 1;
            end
        end
        
         normHist = curHist/sum(curHist);
        
        H_allJoints16{vid}{allJoints}  = normHist;
    end
    
H_jRightElbow16 {vid} = [H_allJoints16{vid}{min(jRightArm)}];
H_jRightWrist16 {vid} = [H_allJoints16{vid}{max(jRightArm)}];

H_jLeftElbow16 {vid} = [H_allJoints16{vid}{min(jLeftArm)}];
H_jLeftWrist16 {vid} = [H_allJoints16{vid}{max(jLeftArm)}];

H_jRightKnee16 {vid} = [H_allJoints16{vid}{min(jRightLeg)}];
H_jRightAnkle16 {vid} = [H_allJoints16{vid}{max(jRightLeg)}];

H_jLeftKnee16 {vid} = [H_allJoints16{vid}{min(jLeftLeg)}];
H_jLeftAnkle16 {vid} = [H_allJoints16{vid}{max(jLeftLeg)}];
        
    
end

%% Reshaping for remaining pipeline

HOJD2D_RightElbow16 = {};
HOJD2D_RightElbow16  = reshape(cell2mat(H_jRightElbow16), (numBin), (numel(vidRange)))';

HOJD2D_RightWrist16 = {};
HOJD2D_RightWrist16  = reshape(cell2mat(H_jRightWrist16), (numBin), (numel(vidRange)))';

HOJD2D_RightKnee16 = {};
HOJD2D_RightKnee16  = reshape(cell2mat(H_jRightKnee16), (numBin), (numel(vidRange)))';

HOJD2D_RightAnkle16 = {};
HOJD2D_RightAnkle16  = reshape(cell2mat(H_jRightAnkle16), (numBin), (numel(vidRange)))';


HOJD2D_LeftElbow16 = {};
HOJD2D_LeftElbow16  = reshape(cell2mat(H_jLeftElbow16), (numBin), (numel(vidRange)))';

HOJD2D_LeftWrist16 = {};
HOJD2D_LeftWrist16  = reshape(cell2mat(H_jLeftWrist16), (numBin), (numel(vidRange)))';

HOJD2D_LeftKnee16 = {};
HOJD2D_LeftKnee16  = reshape(cell2mat(H_jLeftKnee16), (numBin), (numel(vidRange)))';

HOJD2D_LeftAnkle16 = {};
HOJD2D_LeftAnkle16  = reshape(cell2mat(H_jLeftAnkle16), (numBin), (numel(vidRange)))';

% HOJD2D_FullBody16 = {};
% HOJD2D_FullBody16  = reshape(cell2mat(H_jFullBody16), (numBin), (numel(vidRange)))';

%% Export all Histogram data (HOJD2D) for remaining pipeline
save(['25J_RVI_38_HOJD2D_' num2str(numBin) 'bins' '.mat'], 'HOJD2D_RightElbow16', 'HOJD2D_RightWrist16', 'HOJD2D_RightKnee16', 'HOJD2D_RightAnkle16', 'HOJD2D_LeftElbow16', 'HOJD2D_LeftWrist16', 'HOJD2D_LeftKnee16', 'HOJD2D_LeftAnkle16');