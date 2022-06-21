%% Initialization
clear;
close all;

load('C_25J_RVI_38_ProcessedData.mat');
vidRange = 1:38;
numBin = 16; 

theta = (2*pi)/numBin;
vec = [0,1]';

jRightArm = 4:5;
jLeftArm = 7:8;
jRightLeg = 11:12;
jLeftLeg = 14:15;
jFullBody = [1:8 10:21];

H_allJoints16 = {};
H_jRightElbow16 = {};
H_jRightWrist16 = {};
H_jLeftElbow16 = {};
H_jLeftWrist16 = {};
H_jRightKnee16 = {};
H_jRightAnkle16 = {};
H_jLeftKnee16 = {};
H_jLeftAnkle16 = {};

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
        H_allJoints16{vid}{allJoints} = normHist;
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

HOJO2D_RightElbow16 = {};
HOJO2D_RightElbow16  = reshape(cell2mat(H_jRightElbow16), (numBin), (numel(vidRange)))';

HOJO2D_RightWrist16 = {};
HOJO2D_RightWrist16  = reshape(cell2mat(H_jRightWrist16), (numBin), (numel(vidRange)))';

HOJO2D_RightKnee16 = {};
HOJO2D_RightKnee16  = reshape(cell2mat(H_jRightKnee16), (numBin), (numel(vidRange)))';

HOJO2D_RightAnkle16 = {};
HOJO2D_RightAnkle16  = reshape(cell2mat(H_jRightAnkle16), (numBin), (numel(vidRange)))';

HOJO2D_LeftElbow16 = {};
HOJO2D_LeftElbow16  = reshape(cell2mat(H_jLeftElbow16), (numBin), (numel(vidRange)))';

HOJO2D_LeftWrist16 = {};
HOJO2D_LeftWrist16  = reshape(cell2mat(H_jLeftWrist16), (numBin), (numel(vidRange)))';

HOJO2D_LeftKnee16 = {};
HOJO2D_LeftKnee16  = reshape(cell2mat(H_jLeftKnee16), (numBin), (numel(vidRange)))';

HOJO2D_LeftAnkle16 = {};
HOJO2D_LeftAnkle16  = reshape(cell2mat(H_jLeftAnkle16), (numBin), (numel(vidRange)))';

% HOJO2D_FullBody16 = {};
% HOJO2D_FullBody16  = reshape(cell2mat(H_jFullBody16), (numBin), (numel(vidRange)))';


%% Export all Histogram data (HOJD2D) for remaining pipeline
save(['25J_RVI_38_HOJO2D_' num2str(numBin) 'bins' '.mat'], 'HOJO2D_RightElbow16', 'HOJO2D_RightWrist16', 'HOJO2D_RightKnee16', 'HOJO2D_RightAnkle16', 'HOJO2D_LeftElbow16', 'HOJO2D_LeftWrist16', 'HOJO2D_LeftKnee16', 'HOJO2D_LeftAnkle16');

%% Functions
function [rM] = getRotationMat(angle)
    rM = [ cos(angle)   -sin(angle) ; sin(angle)  cos(angle) ];
end