 % load the images
 function MovieCreatCar1(secsPerImage)
 number_images=20;
 images    = cell(number_images,1);
 
 for i=1:1:number_images
 s=num2str(i);    
 images{i} = imread(sprintf('MovingTrajectoryOfvehicle1FIG%s.png',s));
 end
 
 % create the video writer with 1 fps
 writerObj = VideoWriter('myVideoCar1.avi');
 writerObj.FrameRate = 1;
 % set the seconds per image
 secsPerImage = secsPerImage*ones(number_images,1);
 %secsPerImage = 0.004:0.004:(0.004*number_images);
 % open the video writer
 open(writerObj);
 % write the frames to the video
 for u=1:length(images)
     % convert the image to a frame
     frame = im2frame(images{u});
     for v=1:secsPerImage(u) 
         writeVideo(writerObj, frame);
     end
 end
 % close the writer object
 close(writerObj);