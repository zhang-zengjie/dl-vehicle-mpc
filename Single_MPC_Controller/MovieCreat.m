 % load the images
 function MovieCreat()
 number_images=25;
 images    = cell(number_images,1);
 simulations_step=3;
 simulations_step_s=num2str(simulations_step);
 
 for i=1:1:number_images
 iterations_stepss_s=num2str(i);    
 images{i} = imread(sprintf('MovingTrajectorySIMU%sFIG%s.png',simulations_step_s,iterations_stepss_s));
 end
 
 % create the video writer with 1 fps
 writerObj = VideoWriter('myVideoCars.avi');
 writerObj.FrameRate = 5;
 % set the seconds per image
 secsPerImage = 0.004:0.004:(0.004*number_images);
 % open the video writer
 open(writerObj);
 % write the frames to the video
 for u=1:length(images)
     % convert the image to a frame
     frame = im2frame(images{u});
     for v=0.004:0.004:secsPerImage(u) 
         writeVideo(writerObj, frame);
     end
 end
 % close the writer object
 close(writerObj);