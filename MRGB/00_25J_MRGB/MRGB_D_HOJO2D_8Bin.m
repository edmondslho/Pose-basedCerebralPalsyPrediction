%% Initialization
clear;
close all;
load('C_25J_MRGB_ProcessedData.mat');
vidRange = 1:12;
numBin = 8; 
theta = (2*pi)/numBin;
vec = [0,1]';

jRightArm = 4:5;
jLeftArm = 7:8;
jRightLeg = 11:12;
jLeftLeg = 14:15;
jFullBody = [1:8 10:21];

H_allJoints8 = {};
H_jRightElbow8 = {};
H_jRightWrist8 = {};
H_jLeftElbow8 = {};
H_jLeftWrist8 = {};
H_jRightKnee8 = {};
H_jRightAnkle8 = {};
H_jLeftKnee8 = {};
H_jLeftAnkle8 = {};

for i=1:numBin
    rM = getRotationMat(i*theta);
    newVec = rM*vec;
end

binVec = zeros(2,numBin);
theta = (2*pi)/numBin;
vec = [0,1]'; 
for i=1:numBin
    rM = getRotationMat(i*theta);
    binVec(:,i) = rM*vec;
end

%% HOJD2D

for vid = vidRange
    
    for allJoints = jFullBody
        
        frmLength = length(allFinalPose{vid}(1,:))-1;
        frmRange = 1:frmLength;
        
        curHist = zeros(1,numBin);
        for frm = frmRange
            for jID = [allJoints]
                bone = allFinalPose{vid}{frm}(jID,1:2) - allFinalPose{vid}{frm}(pID(jID),1:2);
                
                if norm(bone)>0
                    bone = bone/norm(bone);
                    curHist = curHist + getHistHOJO(bone, binVec, true, true);
                end
            end
        end

        normHist = curHist/sum(curHist);        
        H_allJoints8{vid}{allJoints} = normHist;
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

HOJO2D_RightElbow8 = {};
HOJO2D_RightElbow8  = reshape(cell2mat(H_jRightElbow8), (numBin), (numel(vidRange)))';

HOJO2D_RightWrist8 = {};
HOJO2D_RightWrist8  = reshape(cell2mat(H_jRightWrist8), (numBin), (numel(vidRange)))';

HOJO2D_RightKnee8 = {};
HOJO2D_RightKnee8  = reshape(cell2mat(H_jRightKnee8), (numBin), (numel(vidRange)))';

HOJO2D_RightAnkle8 = {};
HOJO2D_RightAnkle8  = reshape(cell2mat(H_jRightAnkle8), (numBin), (numel(vidRange)))';

HOJO2D_LeftElbow8 = {};
HOJO2D_LeftElbow8  = reshape(cell2mat(H_jLeftElbow8), (numBin), (numel(vidRange)))';

HOJO2D_LeftWrist8 = {};
HOJO2D_LeftWrist8  = reshape(cell2mat(H_jLeftWrist8), (numBin), (numel(vidRange)))';

HOJO2D_LeftKnee8 = {};
HOJO2D_LeftKnee8  = reshape(cell2mat(H_jLeftKnee8), (numBin), (numel(vidRange)))';

HOJO2D_LeftAnkle8 = {};
HOJO2D_LeftAnkle8  = reshape(cell2mat(H_jLeftAnkle8), (numBin), (numel(vidRange)))';

% HOJO2D_FullBody8 = {};
% HOJO2D_FullBody8  = reshape(cell2mat(H_jFullBody8), (numBin), (numel(vidRange)))';


%% Export all Histogram data (HOJD2D) for remaining pipeline
save(['25J_MRGB_HOJO2D_' num2str(numBin) 'bins' '.mat'], 'HOJO2D_RightElbow8', 'HOJO2D_RightWrist8', 'HOJO2D_RightKnee8', 'HOJO2D_RightAnkle8', 'HOJO2D_LeftElbow8', 'HOJO2D_LeftWrist8', 'HOJO2D_LeftKnee8', 'HOJO2D_LeftAnkle8');

%% Functions
function [rM] = getRotationMat(angle)
    rM = [ cos(angle)   -sin(angle) ; sin(angle)  cos(angle) ];
end