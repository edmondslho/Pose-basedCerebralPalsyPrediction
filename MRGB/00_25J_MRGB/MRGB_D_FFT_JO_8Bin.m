%% Initialization

clear;
close all;
load('C_25J_MRGB_ProcessedData.mat');
vidRange = 1:12;
numBin = 8;
pwr = 20;

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


%% FFTJO

for vid = vidRange
    
    for allJoints = jFullBody
        
        frmLength = length(allFinalPose{vid}(1,:))-1;
        frmRange = 1:frmLength;
        
        FFTJO_curHist = zeros(1,numBin);
        
        for frm = frmRange
            for jID = [allJoints]
                bone = allFinalPose{vid}{frm}(jID,1:2) - allFinalPose{vid}{frm}(pID(jID),1:2);
                
                a = bone(1);
                b = bone(2);
                c = a^2 + b^2;
                if (c==0)
                    CosTheta = 0;
                else
                    CosTheta = a/sqrt(a^2 + b^2);
                end
                ThetaInDegrees = acosd(CosTheta);
                Orientation (frm) = ThetaInDegrees;
                
            end
        end
        
        
        Fs = 50;
        T = 1/Fs;
        L = length(Orientation);
        t=(0:L-1)*T;
        
        Y = fft(Orientation);
        P2 = abs(Y/L);
        P1 = P2 (1:L/2+1);
        P1(2:end-1) = 2*P1(2:end-1);
        
        f = Fs*(0:(L/2))/L;        
        maxF = max(f);
        
        for aa = 1:length(f) 
            accBins = 0;
            if f(aa) < maxF*(1^pwr/numBin^pwr)
                bestBin = 1;
                accBins = accBins + (1^pwr/numBin^pwr);
            elseif f(aa) < maxF*(accBins+(2^pwr/numBin^pwr))
                bestBin = 2;
                accBins = accBins + (2^pwr/numBin^pwr);
            elseif f(aa) < maxF*(accBins+(3^pwr/numBin^pwr))
                bestBin = 3;
                accBins = accBins + (3^pwr/numBin^pwr);
            elseif f(aa) < maxF*(accBins+(4^pwr/numBin^pwr))
                bestBin = 4;
                accBins = accBins + (4^pwr/numBin^pwr);
            elseif f(aa) < maxF*(accBins+(5^pwr/numBin^pwr))
                bestBin = 5;
                accBins = accBins + (5^pwr/numBin^pwr);
            elseif f(aa) < maxF*(accBins+(6^pwr/numBin^pwr))
                bestBin = 6;
                accBins = accBins + (6^pwr/numBin^pwr);
            elseif f(aa) < maxF*(accBins+(7^pwr/numBin^pwr))
                bestBin = 7;
                accBins = accBins + (7^pwr/numBin^pwr);
            elseif f(aa) <= maxF
                bestBin = 8;
            end
            
            FFTJO_curHist(bestBin) = FFTJO_curHist(bestBin) + P1(aa);
            
        end
        
        FFTJO_normHist = FFTJO_curHist/sum(FFTJO_curHist);
        H_allJoints8{vid}{allJoints} = FFTJO_normHist;
        
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

FFTJO_RightElbow8 = {};
FFTJO_RightElbow8  = reshape(cell2mat(H_jRightElbow8), (numBin), (numel(vidRange)))';

FFTJO_RightWrist8 = {};
FFTJO_RightWrist8  = reshape(cell2mat(H_jRightWrist8), (numBin), (numel(vidRange)))';

FFTJO_RightKnee8 = {};
FFTJO_RightKnee8  = reshape(cell2mat(H_jRightKnee8), (numBin), (numel(vidRange)))';

FFTJO_RightAnkle8 = {};
FFTJO_RightAnkle8  = reshape(cell2mat(H_jRightAnkle8), (numBin), (numel(vidRange)))';

FFTJO_LeftElbow8 = {};
FFTJO_LeftElbow8  = reshape(cell2mat(H_jLeftElbow8), (numBin), (numel(vidRange)))';

FFTJO_LeftWrist8 = {};
FFTJO_LeftWrist8  = reshape(cell2mat(H_jLeftWrist8), (numBin), (numel(vidRange)))';

FFTJO_LeftKnee8 = {};
FFTJO_LeftKnee8  = reshape(cell2mat(H_jLeftKnee8), (numBin), (numel(vidRange)))';

FFTJO_LeftAnkle8 = {};
FFTJO_LeftAnkle8  = reshape(cell2mat(H_jLeftAnkle8), (numBin), (numel(vidRange)))';

% FFTJO_FullBody8 = {};
% FFTJO_FullBody8  = reshape(cell2mat(H_jFullBody8), (numBin), (numel(vidRange)))';


%% Export all Histogram data (HOJD2D) for remaining pipeline
save(['25J_MRGB_FFT_JO_' num2str(numBin) 'bins' '.mat'], 'FFTJO_RightElbow8', 'FFTJO_RightWrist8', 'FFTJO_RightKnee8', 'FFTJO_RightAnkle8', 'FFTJO_LeftElbow8', 'FFTJO_LeftWrist8', 'FFTJO_LeftKnee8', 'FFTJO_LeftAnkle8');