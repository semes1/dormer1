function varargout = dormer1(varargin)
%代码还是有不足的地方的
% DOMER1 MATLAB code for domer1.fig
%      DOMER1, by itself, creates a new DOMER1 or raises the existing
%      singleton*.
%
%      H = DOMER1 returns the handle to a new DOMER1 or the handle to
%      the existing singleton*.
%
%      DOMER1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DOMER1.M with the given input arguments.
%
%      DOMER1('Property','Value',...) creates a new DOMER1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before domer1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to domer1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help domer1

% Last Modified by GUIDE v2.5 14-May-2024 09:15:06

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @domer1_OpeningFcn, ...
                   'gui_OutputFcn',  @domer1_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before domer1 is made visible.
function domer1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to domer1 (see VARARGIN)

% Choose default command line output for domer1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes domer1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = domer1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


threshold = get(hObject, 'Value'); % 获取滑块的值
set(handles.text8, 'String', num2str(threshold)); % 更新频率值的显示文本

axes(handles.axes3); % 设置当前绘图区域为傅里叶变换后的结果显示区域
 img_src = getimage(handles.axes1); % 从源图像区域获取图像数据
 
% 检查图像是否为 RGB 图像

 if ndims(img_src) == 3

    % 如果是 RGB 图像，则转换为灰度图像
     img_src = rgb2gray(img_src);
 
end

 img_src1 = double(img_src); % 将灰度图像转换为 double 类型
   
img_src2 = abs(fftshift(fft2(img_src1))); % 对灰度图像进行二维快速傅里叶变换，并对结果进行频移和取绝对值
imshow(log(img_src2), []); % 显示傅里叶变换后的结果的对数幅度谱图像
title('傅里叶变换后的结果'); % 设置图像标题

axes(handles.axes5); % 设置当前绘图区域为经过频率域滤波后的结果显示区域
img_src5 = low_filter(fftshift(fft2(img_src1)), threshold); % 对频谱图像进行频率域低通滤波
imshow(log(abs(img_src5)), []); % 显示经过频率域滤波后的结果的对数幅度谱图像
title('经过频率域滤波后的结果'); % 设置图像标题

axes(handles.axes4); % 设置当前绘图区域为滤波结果逆变换到空间域的结果显示区域
new_img = uint8(real(ifft2(ifftshift(img_src5)))); % 对滤波后的频谱图像进行逆变换，并转换为 uint8 类型
imshow(new_img, []); % 显示滤波结果逆变换到空间域的图像
title('滤波结果逆变换到空间域的结果'); % 设置图像标题

set(handles.slider6, 'Enable', 'off'); % 关闭另一个滑块控件

% 更新 text7 中的文本以显示阈值
set(handles.text7, 'String', ['阈值: ', num2str(threshold)]);

% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider7_Callback(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% 获取滑块的值
threshold = get(hObject, 'Value');

% 读取全局变量中保存的图像路径
global fpath;

% 如果图像路径为空，则退出函数
if isempty(fpath)
    return;
end

% 读取图像
img = imread(fpath);

% 将图像二值化处理
bw_img = im2bw(img, threshold);
 %imbalance
% 显示二值化后的图像
axes(handles.axes3);
imshow(bw_img);
title('二值化');
% 更新 text7 中的文本以显示阈值
set(handles.text7, 'String', ['阈值: ', num2str(threshold)]);

% --- Executes during object creation, after setting all properties.
function slider7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%global My_GUI_handles;
axes(handles.axes5);
img_2=getimage(handles.axes1);
average = fspecial('average',[3 3]);%可模板修改
average = imfilter(img_2,average);
imshow(average);
title('3x3均值滤波');

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%global My_GUI_handles;
axes(handles.axes5);
img_2=getimage(handles.axes1);
average = fspecial('average',[5 5]);%鍒涘缓鍧囧?兼护娉㈠櫒锛屾ā鏉垮ぇ灏?3x3
average = imfilter(img_2,average);%杩涜婊ゆ尝鎿嶄綔
imshow(average);
title('5x5均值滤波');

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% axes(handles.axes3);
%     global fpath;
%     img_2=imread(fpath);
%     img_2=rgb2gray(img_2);
%     img_2=imrotate(img_2,90,'nearest');
%     imshow(img_2);
%      title('旋转90度')

     persistent rotationAngle; % 定义一个持久变量，用于保存旋转角度
global fpath;

if isempty(rotationAngle)
    rotationAngle = 90; % 初始旋转角度为顺时针 90 度,
else
    rotationAngle = mod(rotationAngle + 90, 360); % 每次旋转增加 90 度，取模确保在 [0, 360) 范围内
end                                %把+改为-，就会变逆时针

img = imread(fpath);
% img = rgb2gray(img);5转化为灰度图
 img = imrotate(img, rotationAngle, 'nearest');

axes(handles.axes1);
imshow(img);
title(['旋转 ', num2str(rotationAngle), ' 度']);
% handles.img=fliplr(handles.img_2);
% axes(handles.axes3);
% cla;
% imshow(handles.img_2);
% guidata(hObject,handles);
% 
% mysize=size(handles.img_2);
% if numel(mysize)>2
%     updateg4(handles)
% else
%     updateg4_1(handles)
% end 




% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 axes(handles.axes3);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    K=16;
    img_2=histeq(img_2,K);
    imshow(img_2);
    title('直方图均衡')
    

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

 axes(handles.axes3);
    global fpath;
    img_2=imread(fpath);
    img_2=im2bw(img_2,0.5);
    imshow(img_2);
    title('二值化图像');




% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 axes(handles.axes5);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    imshow(img_2);
     title('灰度图')
% 将灰度图像显示到 axes5 中


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 axes(handles.axes3);
    global fpath ;%
    img_2=imread(fpath);%
%     img_2=rgb2gray(img_2);
    img_2=imnoise(img_2,'gaussian');
    imshow(img_2);
    title('加入高斯噪声')
    
 img_6=rgb2gray(img_2);
axes(handles.axes4);  % 切换到 axes4
imshow(img_6);  % 显示图像
title('加入高斯噪声的灰度图像');  % 添加标题

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% global fpath 
% J = imnoise(fpath, 'salt & pepper', 0.02);
% axes(handles.axes3)%椒盐噪声
% imshow(J)


 axes(handles.axes3);
    global fpath;
    img_2=imread(fpath);
    
    img_2=imnoise(img_2,'salt & pepper',0.06);
    imshow(img_2);
     title('加入椒盐噪声后')
  img_4=rgb2gray(img_2);
axes(handles.axes4);  % 切换到 axes4
imshow(img_4);  % 显示图像
title('加入椒盐噪声后的灰度图像');  % 添加标题


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



clc; %清屏
 [filename,pathname]=uigetfile(...
        {'*.bmp;*.jpg;*.png;*.jpeg','ImageFiles(*.bmp,*.jpg,*.png,*.jpeg)';...
            '*.*','AllFiles(*.*)'},...
            'Pickanimage');
% if isequal(filename,0) || isequal(pathname,0)
    axes(handles.axes1);
    global fpath;
    fpath=[pathname filename];%打开图片
    img_1=imread(fpath);
    
    whos img_1;%在工作空间列出变量
    imshow(img_1);
    title('原始图像');

% 获取图像数据矩阵或数组的大小
[rows, cols, channels] = size(img_1);

% 显示图像数据矩阵或数组的行列维数
fprintf('图像数据矩阵或数组的大小为：%d 行 x %d 列 x %d 通道\n', rows, cols, channels);
 img_5=rgb2gray(img_1);
axes(handles.axes5);  % 切换到 axes4
imshow(img_5);  % 显示图像
title('原始图像的灰度图像');  % 添加标题
% % 显示图像数据矩阵
% disp('图像数据矩阵:');
% disp(img_1);

%  % 直接在gui的axes3界面显示变量
% workspace_info = whos;
% info_str = '';
% for i = 1:numel(workspace_info)
%     info_str = [info_str, sprintf('%s: %d sbyte\n', workspace_info(i).name, workspace_info(i).bytes)];
% end
% 
% axes(handles.axes3);
% cla; % Clear axes3
% text(0.1, 0.5, info_str, 'FontSize', 10); % Display text in axes3
% axis off; % Turn off axis
%  title('图像数据');


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close all;%退出程序

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 [filename,pathname]=uiputfile({'*.bmp','BMPfiles';'*.jpg;','JPGfiles'},'PickanImage');
        if isequal(filename,0)||isequal(pathname,0)
            return;
        else
            h=getframe(handles.axes1);  
            imwrite(h.cdata,[pathname,filename]);  
        end

% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% 清除图像区域内容
axes(handles.axes1);%修改axes3,axes4,axes5指定清除哪个区域的图片
cla;

% 更新其他相关的图像或控件
% updateg4(handles);

% 保存 GUI 数据
guidata(hObject, handles);




% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
persistent flipHorizontal; % 定义一个持久变量，用于保存图像是否水平翻转的状态
global fpath;

if isempty(flipHorizontal)
    flipHorizontal = false; % 初始状态为未翻转
else
    flipHorizontal = ~flipHorizontal; % 切换翻转状态
end

img = imread(fpath);
% img = rgb2gray(img);

if flipHorizontal
    img = flip(img, 2); % 水平翻转图像，把2改为1为垂直翻转
end

axes(handles.axes1);
imshow(img);
if flipHorizontal
    title('水平翻转');
else
    title('未翻转');
end


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 获取在 axes1 中显示的图像数据
img_data = getimage(handles.axes1);

% 分离 RGB 通道并反转颜色
r = img_data(:,:,1);
g = img_data(:,:,2);
b = img_data(:,:,3);
r = 256 - r;
g = 256 - g;
b = 256 - b;

% 将反转后的 RGB 通道重新组合成新的图像
img_inverted = cat(3, r, g, b);

% 在 axes3 中显示处理后的图像
axes(handles.axes3);
cla; 
imshow(img_inverted);
 title('反色后的图像');
% 可能还有其他与 GUI 相关的更新操作
%updateg4(handles);

% 保存 GUI 数据
guidata(hObject, handles);


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 axes(handles.axes3);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    img_2=im2double(img_2);
    img_2=fft2(img_2);
    img_2=fftshift(img_2);
    img_2=abs(img_2);
    img_2=log(img_2+1);
    imshow(img_2,[]);
    title('傅里叶变换（频谱图）')
    

% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes3);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    img_2=imnoise(img_2,'salt & pepper',0.06);
     img_2=im2double(img_2);
    img_2=fft2(img_2);
    img_2=fftshift(img_2);
    img_2=abs(img_2);
    img_2=log(img_2+1);
    imshow(img_2,[]);
    title('加入噪声后的傅里叶变换（频谱图）')




% --- Executes on button press in pushbutton30.
function pushbutton30_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes5);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    img_2=imnoise(img_2,'gaussian');
   
   [height,width] = size(img_2);
J = img_2;
    conv = zeros(5,5);
sigma = 1;
sigma_2 = sigma * sigma;
sum = 0;
for i = 1:5
    for j = 1:5
        conv(i,j) = exp((-(i - 3) * (i - 3) - (j - 3) * (j - 3)) / (2 * sigma_2)) / (2 * 3.14 * sigma_2);
        sum = sum + conv(i,j);
    end
end
conv = conv./sum;
for i = 1:height
    for j = 1:width
        sum = 0;
        for k = 1:5
            for m = 1:5
                if (i - 3 + k) > 0 && (i - 3 + k) <= height && (j - 3 + m) > 0 && (j - 3 + m) < width
                    sum = sum + conv(k,m) * img_2(i - 3 + k,j - 3 + m);
                end
            end
        end
        J(i,j) = sum;
    end
end
imshow(J,[])
title('高斯滤波')


% --- Executes on button press in pushbutton31.
function pushbutton31_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 axes(handles.axes5);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    img_2=imnoise(img_2,'salt & pepper',0.06);
    img_2=medfilt2(img_2);
    imshow(img_2);
     title('中值滤波')


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton35.
function pushbutton35_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes3);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    img_2=edge(img_2,'canny');
    imshow(img_2,[]);
    title('canny边缘检测')


% --- Executes on button press in pushbutton36.
function pushbutton36_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes3);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    img_2=edge(img_2,'sobel');
    imshow(img_2,[]);
    title('Sobel边缘检测')

% --- Executes on button press in pushbutton37.
function pushbutton37_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton37 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 axes(handles.axes3);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    img_2=edge(img_2,'log');
    imshow(img_2,[]);
    title('Log边缘检测')


% --- Executes on button press in pushbutton38.
function pushbutton38_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton38 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 axes(handles.axes3);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    img_2=edge(img_2,'roberts');
    imshow(img_2,[]);
    title('Roberts边缘检测')

% --- Executes on button press in pushbutton39.
function pushbutton39_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton39 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 axes(handles.axes3);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    img_2=edge(img_2,'prewitt');
    imshow(img_2,[]);
    title('Prewitt边缘检测')


% --- Executes on slider movement.
function slider8_Callback(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global T;
% global My_GUI_handles;
T=get(hObject,'Value');
% set(handles.text5,'String',num2str(T));
set(handles.edit5,'String',num2str(T));%
% img_src=getimage(My_GUI_handles.axes_src);
% img_src=im2gray(img_src);
% robertsmax = roberts_suanzi(img_src);
set(handles.slider8,'Max',6);%更改0.5调整精度

% --- Executes during object creation, after setting all properties.
function slider8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end





% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton44.
function pushbutton44_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% % 在 axes4 中显示灰度图像
% axes(handles.axes4);
% imshow(img_gray);
% % 在 axes1 中显示原始图像
% axes(handles.axes1);
% imshow(img_src)；


global T;
% 获取当前坐标轴 axes1 中的图像
img_src = getimage(handles.axes1);
% 将 RGB 图像转换为灰度图像
img_gray = rgb2gray(img_src);
% 对灰度图像进行 Prewitt 边缘检测
img_edges = edge(img_gray, 'prewitt', T, 'horizontal', 'nothinning');
% 在 axes3 中显示 Prewitt 边缘检测结果
axes(handles.axes3);
imshow(img_edges);
 title('prewitt水平方向边缘检测');



% --- Executes on button press in pushbutton45.
function pushbutton45_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton45 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%global My_GUI_handles;
global T;
axes(handles.axes4);
img_src=getimage(handles.axes1);
img_gray = rgb2gray(img_src);
img_src_prewitt=edge(img_gray,"prewitt",T,"vertical","nothinning");
imshow(img_src_prewitt);
 title('prewitt垂直方向边缘检测');
% --- Executes on button press in pushbutton46.
function pushbutton46_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton46 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global T;
axes(handles.axes5);
img_src=getimage(handles.axes1);
img_gray = rgb2gray(img_src);
% img_src=im2gray(img_src);
img_src_prewitt=edge(img_gray,"prewitt",T,"both","nothinning");
imshow(img_src_prewitt);
 title('prewitt双方向边缘检测');


% --- Executes on button press in pushbutton41.
function pushbutton41_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 axes(handles.axes3);
    global fpath;
    img_2 = imread(fpath);
    img_2 = rgb2gray(img_2);
    img_2 = edge(img_2, 'sobel', [], 'horizontal'); % 在这里使用 'horizontal' 选项
    imshow(img_2, []);
    title('Sobel水平方向边缘检测');

% --- Executes on button press in pushbutton42.
function pushbutton42_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 axes(handles.axes4);
    global fpath;
    img_2 = imread(fpath);
    img_2 = rgb2gray(img_2);
    img_2 = edge(img_2, 'sobel', [], 'vertical'); % 在这里使用 'vertical' 选项
    imshow(img_2, []);
    title('Sobel垂直方向边缘检测');


% --- Executes on button press in pushbutton43.
function pushbutton43_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes5);
    global fpath;
    img_2 = imread(fpath);
    img_2 = rgb2gray(img_2);
    img_2 = edge(img_2, 'sobel', [], 'both'); % 在这里使用 'both' 选项
    imshow(img_2, []);
    title('Sobel双方向边缘检测');


% --- Executes on button press in pushbutton40.
function pushbutton40_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton40 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%global My_GUI_handles;

img_2 = getimage(handles.axes1);%img_src改为img_2检测噪声图片
% 将 RGB 图像转换为灰度图像
img_gray = rgb2gray(img_2);%img_src改为img_2检测噪声图片
% 在 axes4 中显示灰度图像
axes(handles.axes4);

imshow(img_gray);
title('原图像灰度图像');
% 对灰度图像进行 Roberts 边缘检测
global T;
img_edges = edge(img_gray, 'roberts', T, 'nothinning');
% 在 axes3 中显示 Roberts 边缘检测结果
axes(handles.axes3);
imshow(img_edges); 
title('roberts边缘检测');


axes(handles.axes5);
imhist(img_2);%img_src改为img_2检测噪声图片
title('roberts边缘检测直方图');
% % 在 axes5 中显示原始图像
% axes(handles.axes5);
% imshow(img_src);
%   title('roberts边缘检测');


function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double
global T;
edit_edge=get(handles.edit5,'String');
T=str2double(edit_edge);

% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton47.
function pushbutton47_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton47 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 
%     img = imread('D:\matlabs\数字图像处理实验\实验图片\zg.jpg');
%     gray = rgb2gray(img);
%     r = img(:,:,1);
 % 打开文件选择对话框
    [filename, pathname] = uigetfile({'*.jpg;*.png;*.bmp', 'Image Files (*.jpg, *.png, *.bmp)'}, '选择图像文件');
    if isequal(filename, 0) || isequal(pathname, 0)
        % 用户取消选择图像文件
        return;
    end
    
    % 读取所选图像文件
    img = imread(fullfile(pathname, filename));
    gray = rgb2gray(img);
    r = img(:,:,1);
    % 显示灰度图像在 axes1
    axes(handles.axes1);
    imshow(gray);
     title('灰度图像');
    % 显示灰度图像直方图在 axes3
    axes(handles.axes4);
    imhist(gray);
     title('灰度图像直方图');
    % 显示红色通道图像在 axes4
    axes(handles.axes3);
    imshow(r);
     title('红色通道图像');
    % 显示红色通道直方图在 axes5
    axes(handles.axes5);
    imhist(r);
 title('红色通道直方图');


% --- Executes on button press in pushbutton48.
function pushbutton48_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton48 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 % 打开文件选择对话框，让用户选择要打开的图像文件
    [filename, pathname] = uigetfile({'*.jpg;*.png;*.bmp', 'Image Files (*.jpg, *.png, *.bmp)'}, '选择图像文件');
    
    % 检查用户是否取消了选择
    if isequal(filename, 0) || isequal(pathname, 0)
        % 用户取消选择图像文件，直接返回
        return;
    end
    
    % 读取用户选择的图像文件
    img = imread(fullfile(pathname, filename));
    
    % 提取图像的灰度图像以及红、绿、蓝通道
    gray = rgb2gray(img);
    r = img(:,:,1);
    g = img(:,:,2);
    b = img(:,:,3);
    
    % 在指定的 axes 中显示每个通道的图像
    axes(handles.axes1);
    imshow(gray);
    title('灰度图像');
    
    axes(handles.axes5);
    imshow(r);
    title('红色通道');
    
    axes(handles.axes3);
    imshow(g);
    title('绿色通道');
    
    axes(handles.axes4);
    imshow(b);
    title('蓝色通道');


% --- Executes on button press in pushbutton49.
function pushbutton49_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton49 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 打开文件选择对话框，让用户选择要显示的图片文件
    
    % 打开文件选择对话框，让用户选择要显示的图片文件
    [filename, pathname] = uigetfile({'*.jpg', 'JPEG Files (*.jpg)'}, '选择要打开的 JPG 图片文件');
    
    % 检查用户是否取消了选择
    if isequal(filename, 0) || isequal(pathname, 0)
        % 用户取消选择图像文件，直接返回
        return;
    end
    
    % 读取用户选择的 JPG 图片文件
    img_src1 = imread(fullfile(pathname, filename));
    
    % 在 axes1 中显示用户选择的 JPG 图片
    axes(handles.axes1);
    imshow(img_src1);
    title('JPG 图片');
    
    % 将生成的 BMP 和 TIF 图片存储在指定文件夹
    output_folder = 'D:\matlabs\数字图像处理实验\实验图片';
    [~, filebase, ~] = fileparts(filename);
    bmp_filename = fullfile(output_folder, [filebase, '.bmp']);
    tif_filename = fullfile(output_folder, [filebase, '.tif']);
    imwrite(img_src1, bmp_filename, 'bmp');
    imwrite(img_src1, tif_filename, 'tif');
    
    % 在 axes3 和 axes4 中显示另存的 BMP 和 TIF 图片
%      img_src1 = imread(jpg_filename);
    img_src2 = imread(bmp_filename); 
    img_src3 = imread(tif_filename);
   %bmp_filename 和 tif_filename 分别是 BMP 和 TIF 格式图像文件的完整路径和名称
    axes(handles.axes3);
    imshow(img_src2);
    title('BMP 图片');
    
    axes(handles.axes4);
    imshow(img_src3);
    title('TIF 图片');
    
    
% 提取图像的基本信息并显示
% 提取 img_src1 的基本信息
info1 = imfinfo(fullfile(pathname, filename));
disp('img_src1 的基本信息:');
disp(info1);

% 提取 img_src2 的基本信息
info2 = imfinfo(bmp_filename);
disp('img_src2 的基本信息:');
disp(info2);

% 提取 img_src3 的基本信息
info3 = imfinfo(tif_filename);
disp('img_src3 的基本信息:');
disp(info3);


% --- Executes on button press in pushbutton54.
function pushbutton54_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton54 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes3);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    img_2=imnoise(img_2,'salt & pepper',0.06);
    img_2=double(img_2);
    img_2=fft2(img_2);
    img_2=fftshift(img_2);
    
    [M,N,~]=size(img_2);
nn=2;
d0=50;
m=floor(M/2);n=floor(N/2);
for i=1:M
    for j=1:N
        d=sqrt((i-m)^2+(j-n)^2);
        h=1/(1+0.414*(d/d0)^(2*nn));
        result(i,j)=h*img_2(i,j);
    end
end
result=ifftshift(result);
J2=ifft2(result);
J3=uint8(real(J2));
imshow(J3);
title('低通滤波');


% --- Executes on button press in pushbutton55.
function pushbutton55_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton55 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes4);
    global fpath;
    img_2=imread(fpath);
    img_2=rgb2gray(img_2);
    img_2=imnoise(img_2,'salt & pepper',0.06);
     img_2=double(img_2);
    img_2=fft2(img_2);
    img_2=fftshift(img_2);
    
    [M,N,~]=size(img_2);
nn=2;
d0=50;
m=floor(M/2);n=floor(N/2);
for i=1:M
    for j=1:N
        d=sqrt((i-m)^2+(j-n)^2);
        h=1/(1+0.414*(d0/d)^(2*nn));
        result(i,j)=h*img_2(i,j);
    end
end

result=ifftshift(result);
J2=ifft2(result);
J3=uint8(real(J2));
imshow(J3);
title('高通滤波图');
% 

% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% 

% handles.output = hObject;%添加背景
% background = axes('units','normalized','pos',[0 0 1 1]);
% uistack(background,'bottom');
% bag = imread('C:\Users\llyy\Desktop\数字图像处理实验\实验图片\背景图.png');   % 默认图片
% image(bag)
% set(background,'HandleVisibility','off','Visible','off');


% --- Executes on key press with focus on pushbutton2 and none of its controls.
function pushbutton2_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% % --- Executes on button press in pushbutton58.
% function pushbutton58_Callback(hObject, eventdata, handles)
% % hObject    handle to pushbutton58 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% % global My_GUI_handles;
%  global fpath;
% global houghpeaks_Slope_type;
%  
% global houghpeaks_Peak_multiples;
%  
% global houghlines_Line_distance;
%  
% global houghlines_Line_length;
%  
%  
%  
% img_src=getimage(handles.axes1);
%  
% I1=imresize(img_src,[300 300]);
%   I1=rgb2gray(I1);
% % I=im2gray(I1);
%  
% %rotI = imrotate(I,33,'crop');
%  
% BW = edge( fpath,'canny');%可以设置使用其他算子
%  
% my_mask = mask_rectangle(BW);
%  
% [H,T,R] = hough(my_mask);
%  
%  
%  
% axes(handles.axes3);
%  
% imshow(H,[],'XData',T,'YData',R,'InitialMagnification','fit');
%  
% xlabel('\theta'), ylabel('\rho');
%  
% hold on
%  
% % axis on, axis normal, hold on;
%  
% P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
%  
% x = T(P(:,2));
%  
% y = R(P(:,1));
%  
% plot(x,y,'s','color','white');
%  
%  
%  
% % Find lines and plot them
%  
% q  = houghpeaks(H,houghpeaks_Slope_type,'threshold',ceil(houghpeaks_Peak_multiples*max(H(:))));%4代表四种斜率的所有线段*****建议变量10和0.7
%  
% lines = houghlines(my_mask,T,R,q,'FillGap',houghlines_Line_distance,'MinLength',houghlines_Line_length);%*********建议变量3和20一起
%  
% axes(handles.axes4);
%  
% imshow(I1),hold on
%  
% max_len = 0;
%  
% for k = 1:length(lines)
%  
%    xy = [lines(k).point1; lines(k).point2];
%  
%    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
%  
%  
%  
%    % Plot beginnings and ends of lines
%  
%    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%  
%    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
%  
%  
%  
%    % Determine the endpoints of the longest line segment
%  
%    len = norm(lines(k).point1 - lines(k).point2);
%  
%    if ( len > max_len)
%  
%       max_len = len;
%  
%       xy_long = xy;
%  
%    end
%  
% end
%  
% % highlight the longest line segment
%  
% plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','cyan');
%  
%  
%  
% axes(handles.axes5);
%  
% imshow(my_mask);


% --- Executes on button press in pushbutton59.
function pushbutton59_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton59 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


  % 打开文件选择对话框，让用户选择要检测的图片文件
    [filename, pathname] = uigetfile({'*.jpg;*.png;*.bmp;*.tif', 'Image Files (*.jpg, *.png, *.bmp, *.tif)'}, '选择要检测的图片文件');
    
    % 检查用户是否取消了选择
    if isequal(filename, 0) || isequal(pathname, 0)
        % 用户取消选择图像文件，直接返回
        return;
    end
    
    % 读取用户选择的图片文件
    img_src = imread(fullfile(pathname, filename));
    
    % 将图像调整为200x200大小
    img_src = imresize(img_src, [200, 200]);
    
    % 转换为灰度图像
    img_gray = rgb2gray(img_src);
    
    % 使用Canny边缘检测算法
    BW = edge(img_gray, 'canny');
    
    % 计算霍夫变换
    [H, T, R] = hough(BW);
    
    % 在 axes3 中显示霍夫变换结果
    axes(handles.axes3);
    imshow(H, [], 'XData', T, 'YData', R, 'InitialMagnification', 'fit');
    xlabel('\theta'), ylabel('\rho');
    axis on, axis normal, hold(handles.axes3, 'on');
    
    % 在霍夫变换结果上标记峰值点
    P = houghpeaks(H, 5, 'threshold', ceil(0.3 * max(H(:))));
    x = T(P(:,2));
    y = R(P(:,1));
    plot(handles.axes3, x, y, 's', 'color', 'white');
    
    % 查找直线并绘制
    q = houghpeaks(H, 10, 'threshold', ceil(0.3 * max(H(:))));
    lines = houghlines(BW, T, R, q, 'FillGap', 5, 'MinLength', 7);
    max_len = 0;
    
    % 在 axes4 中显示原始图像
    axes(handles.axes4);
    imshow(img_src);
    hold(handles.axes4, 'on');
    
    for k = 1:length(lines)
        xy = [lines(k).point1; lines(k).point2];
        plot(handles.axes4, xy(:,1), xy(:,2), 'LineWidth', 2, 'Color', 'green');
        
        % 绘制直线的起点和终点
        plot(handles.axes4, xy(1,1), xy(1,2), 'x', 'LineWidth', 2, 'Color', 'yellow');
        plot(handles.axes4, xy(2,1), xy(2,2), 'x', 'LineWidth', 2, 'Color', 'red');
        
        % 计算最长线段的端点
        len = norm(lines(k).point1 - lines(k).point2);
        if ( len > max_len)
            max_len = len;
            xy_long = xy;
        end
    end
    
    % 在 axes4 中突出显示最长线段
    plot(handles.axes4, xy_long(:,1), xy_long(:,2), 'LineWidth', 2, 'Color', 'cyan');
    title(handles.axes4, '检测结果');
    
    % 获取并显示直线检测结果信息
    disp_info(length(lines), max_len);


% 函数：disp_info
% 用于显示直线检测结果的信息
function disp_info(num_lines, max_length)
    disp(['检测到直线数量：', num2str(num_lines)]);
    disp(['最长直线长度：', num2str(max_length)]);



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton60.
function pushbutton60_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton60 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.output = hObject;%添加背景
background = axes('units','normalized','pos',[0 0 1 1]);
uistack(background,'bottom');
bag = imread('C:\Users\llyy\Desktop\2021900372数字图像处理代码\背景图.png');   % 默认图片
image(bag)
set(background,'HandleVisibility','off','Visible','off');
