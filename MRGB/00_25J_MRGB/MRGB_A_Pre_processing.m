%% Pre-processing the data
clear;
vidRange = 1:12;
jointRange = [1:15, 20:25];
frmRange = 0:998;
idX = [1:length(frmRange)];

for vid=vidRange
    allPose{vid} = {};
    
    jTrajectory{vid} = zeros(25,3,length (frmRange));

        
      for frm=frmRange
          fname = sprintf('MRGBD/MINI-RGB_Export_%02i_%012i_keypoints.json', vid, frm);
          val = jsondecode(fileread(fname));

        pose = reshape(val.people(1).pose_keypoints_2d, 3, 25)';
        
        allPose{vid}{frm+1} = pose;
                
        jTrajectory{vid}(:,:,frm+1) = pose;
                   
        iPose {vid} {frm+1} = {};
    end
    
     
    for jID = jointRange
    
        allConfi = reshape(jTrajectory{vid}(jID,3,:), 1,length (frmRange));
        allX = reshape(jTrajectory{vid}(jID,1,:), 1,length (frmRange));
        allY = reshape(jTrajectory{vid}(jID,2,:), 1,length (frmRange));
        M = mean(allConfi);


        exIdx = find(allConfi>=(M-0.07));

        inPt = allX(exIdx);
        inIdx = idX(exIdx);
        xx = [1:length(idX)];
        
        INTxx = interp1(inIdx,inPt,xx, 'makima');
        
        SmoothXX = movmean(INTxx,5); 

        inPt = allY(exIdx);
        inIdx = idX(exIdx);
        yy = [1:length(idX)];
        
        INTyy = interp1(inIdx,inPt,yy, 'makima');
        
        SmoothYY = movmean(INTyy,5);


        correctedJoint = zeros(25,3,length (frmRange));
        correctedJoint(jID,1,:) = SmoothXX;
        correctedJoint(jID,2,:) = SmoothYY;
        correctedJoint(jID,3,:) = allConfi;   

            for frm=frmRange 
            iPose {vid} {frm+1} {jID} = (correctedJoint((jID),:,(frm+1)));
            end
    
    end    
    
    
end

%% Exporting the Corrected Pose Data

 save('A_25J_MRGB_interpPoseData.mat', 'iPose');