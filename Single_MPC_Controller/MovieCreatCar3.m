 % load the images
 function MovieCreatCar3()
 number_images=25;
 images    = cell(number_images,1);
 
 for i=1:1:number_images
 s=num2str(i);    
 images{i} = imread(sprintf('MovingTrajectoryOfvehicle3FIG%s.png',s));
 end
 
 % create the video writer with 1 fps
 writerObj = VideoWriter('myVideoCar3.avi');
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