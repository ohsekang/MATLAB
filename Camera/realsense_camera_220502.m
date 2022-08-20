%%
% webcam list 검색
webcamlist

%%
% intel realsense RGB camera 선택
cam = webcam('Intel(R) RealSense(TM) 515 RGB');
cam.Resolution = '1920x1080';

%%
% 실시간 영상 스트리밍
preview(cam)

%%
% 스트리밍 종료
closePreview(cam)

%%
% image 캡쳐 후 img 변수에 저장
img = snapshot(cam);

%%
% 해당 이미지 띄우기
imshow(img)

%%
% cam 객체 삭제
clear('cam');
