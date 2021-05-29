function varargout = GUI(varargin)
% GUI MATLAB code for GUI.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI

% Last Modified by GUIDE v2.5 18-Jan-2021 06:31:17

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_OutputFcn, ...
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


% --- Executes just before GUI is made visible.
function GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI (see VARARGIN)
axes(handles.axes4);
imshow('1268183.jpg')
% Choose default command line output for GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


%---------------------- BROWSE FUNCTIONS ----------------------------------
function pushbutton16_Callback(hObject, eventdata, handles)
browsebutton_Callback(hObject, eventdata, handles);

function browsebutton_Callback(hObject, eventdata, handles)
cla(handles.axes2);
cla(handles.axes3);
[filename,path,ext] =uigetfile({'*.tif;*.jpg;*.jpeg;*.png','All Files'},'Select Image');
if isequal(filename,0)
    opts.WindowStyle = 'non-modal'; opts.Interpreter = 'tex';
    err = errordlg('\bf\color{red} NO IMAGE SELECTED, PLEASE SELECT AN IMAGE','ERROR',opts);
    set(handles.buttonspanel,'visible','off')
    set(handles.uipanel10,'visible','off')
    set(handles.uipanel11,'visible','off')
    set(handles.uipanel12,'visible','off')
    set(handles.axes1text,'visible','off')
    set(handles.axes2text,'visible','off')
    set(handles.axes3text,'visible','off')
    set(handles.sobel_panel,'visible','off')
    set(handles.laplace_panel,'visible','off')
    set(handles.sp_panel,'visible','off')
    set(handles.periodic_panel,'visible','off')
    set(handles.uipanel20,'visible','off')
    set(handles.removeperiodicnoisepanel,'visible','off');
    set(handles.pushbutton16,'visible','on')
else
    try        
       img_name = strcat(path,filename);
       img = imread(img_name);
       set(handles.uipanel10,'visible','on')
       set(handles.uipanel11,'visible','on')
       set(handles.uipanel12,'visible','on')
       set(handles.axes1text,'visible','on')
       set(handles.axes2text,'visible','on')
       set(handles.axes3text,'visible','on')
       set(handles.sobel_panel,'visible','off')
       set(handles.laplace_panel,'visible','off')
       set(handles.sp_panel,'visible','off')
       set(handles.periodic_panel,'visible','off')
       set(handles.uipanel20,'visible','off')
       set(handles.removeperiodicnoisepanel,'visible','off');
       axes(handles.axes1);
       imshow(img)
       set(handles.axes2text,'String',"...");
       set(handles.axes3text,'String',"...");

       if (size(img,3)==3)
             handles.gray_image = rgb2gray(img);
       else            
            handles.gray_image = img;
       end
        guidata(hObject,handles);        
        set(handles.buttonspanel,'visible','on')
        set(handles.pushbutton16,'visible','off')
    catch 
       opts.WindowStyle = 'non-modal'; opts.Interpreter = 'tex';
       errordlg(['\bf\color{red} THERE IS A PROBLEM WITH THE SELECTED IMAGE, PLEASE SELECT ANOTHER IMAGE'],'Load Error',opts);
       set(handles.buttonspanel,'visible','off')
       set(handles.uipanel10,'visible','off')
       set(handles.uipanel11,'visible','off')
       set(handles.uipanel12,'visible','off')
       set(handles.axes1text,'visible','off')
       set(handles.axes2text,'visible','off')
       set(handles.axes3text,'visible','off')
       set(handles.sobel_panel,'visible','off')
       set(handles.laplace_panel,'visible','off')
       set(handles.sp_panel,'visible','off')
       set(handles.periodic_panel,'visible','off')
       set(handles.uipanel20,'visible','off')
       set(handles.removeperiodicnoisepanel,'visible','off');
       set(handles.pushbutton16,'visible','on')
       return;
    end
end


%------------------------- HISTOGRAM --------------------------------------
function histogrambutton_Callback(hObject, eventdata, handles)
set(handles.sobel_panel,'visible','off')
set(handles.laplace_panel,'visible','off')
set(handles.sp_panel,'visible','off')
set(handles.periodic_panel,'visible','off')
set(handles.removeperiodicnoisepanel,'visible','off');
set(handles.uipanel20,'visible','off')

gray_img = handles.gray_image;
axes(handles.axes2);
imshow(gray_img);
set(handles.axes2text,'String',"Gray Image");
axes(handles.axes3);
imhist(gray_img),axis tight
set(handles.axes3text,'String',"Gray Image Histogram");


%---------------------- HISTOGRAM EQUALIZATION ----------------------------
function histeqbutton_Callback(hObject, eventdata, handles)
set(handles.sobel_panel,'visible','off')
set(handles.laplace_panel,'visible','off')
set(handles.sp_panel,'visible','off')
set(handles.periodic_panel,'visible','off')
set(handles.removeperiodicnoisepanel,'visible','off');
set(handles.uipanel20,'visible','off')

gray_img = handles.gray_image;
axes(handles.axes2);
eq_image = histeq(gray_img);
imshow(eq_image);
set(handles.axes2text,'String',"Equalized Gray Image");
axes(handles.axes3);
imhist(eq_image), axis tight
set(handles.axes3text,'String',"Equalized Histogram");


%------------------------------ SOBEL -------------------------------------
function thresholdslider_Callback(hObject, eventdata, handles)
global thresh 
thresh = get(hObject,'Value');
text = strcat('Threshold = ',num2str(thresh));
set(handles.thresholdText,'String',text);
sobelbutton_Callback(hObject, eventdata, handles);

function thresholdslider_CreateFcn(hObject, eventdata, handles)
global thresh 
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
thresh = get(hObject,'Value');

function direction_popupmenu_Callback(hObject, eventdata, handles)
global direction
contents = cellstr(get(hObject,'String'));
popchoice = contents{get(hObject,'Value')};
if(strcmp(popchoice,'XY Direction'))
    direction=1;
elseif(strcmp(popchoice,'Horizontal'))
    direction = 2;
elseif(strcmp(popchoice,'Vertical'))
    direction = 3;
end
sobelbutton_Callback(hObject, eventdata, handles);

function direction_popupmenu_CreateFcn(hObject, eventdata, handles)
global direction
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
direction = 1;

function sobelbutton_Callback(hObject, eventdata, handles)
global thresh direction
set(handles.sobel_panel,'visible','on')
set(handles.laplace_panel,'visible','off')
set(handles.sp_panel,'visible','off')
set(handles.periodic_panel,'visible','off')
set(handles.removeperiodicnoisepanel,'visible','off');
set(handles.uipanel20,'visible','off')

gray_img = handles.gray_image;
axes(handles.axes2);
imshow(gray_img);
set(handles.axes2text,'String',"Gray Image");
if(direction==1)
    edge_s = edge(gray_img,'sobel',thresh);
elseif(direction==2)
    edge_s = edge(gray_img,'sobel',thresh,'horizontal');
elseif(direction==3)
    edge_s = edge(gray_img,'sobel',thresh,'vertical');
end
axes(handles.axes3);
imshow(edge_s);
set(handles.axes3text,'String',"Sobel Filter");


%------------------------------ LAPLACE -----------------------------------
function alphaslider_Callback(hObject, eventdata, handles)
global alpha
alpha = get(hObject,'Value');
text = strcat('Alpha = ',num2str(alpha));
set(handles.alphavalue,'String',text);
laplacebutton_Callback(hObject, eventdata, handles);

function alphaslider_CreateFcn(hObject, eventdata, handles)
global alpha
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
alpha = get(hObject,'Value');

function laplacedensityslider_Callback(hObject, eventdata, handles)
global laplacedensity
laplacedensity = get(hObject,'Value');
text = strcat('Scaling Factor = ',num2str(laplacedensity));
set(handles.densityvalue,'String',text);
laplacebutton_Callback(hObject, eventdata, handles);

function laplacedensityslider_CreateFcn(hObject, eventdata, handles)
global laplacedensity
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
laplacedensity = get(hObject,'Value');

function laplacebutton_Callback(hObject, eventdata, handles)
global alpha laplacedensity;
set(handles.laplace_panel,'visible','on')
set(handles.sobel_panel,'visible','off')
set(handles.sp_panel,'visible','off')
set(handles.periodic_panel,'visible','off')
set(handles.removeperiodicnoisepanel,'visible','off');
set(handles.uipanel20,'visible','off')

gray_img = handles.gray_image;
axes(handles.axes2);
imshow(gray_img);
set(handles.axes2text,'String',"Gray Image");
lap=fspecial('laplacian',alpha);
img_lap=filter2(lap,gray_img);
axes(handles.axes3);
imshow(img_lap/laplacedensity);
set(handles.axes3text,'String',"Laplace Filter");


%------------------------------ FOURIER -----------------------------------
function fourierbutton_Callback(hObject, eventdata, handles)
set(handles.sobel_panel,'visible','off')
set(handles.laplace_panel,'visible','off')
set(handles.sp_panel,'visible','off')
set(handles.periodic_panel,'visible','off')
set(handles.removeperiodicnoisepanel,'visible','off');
set(handles.uipanel20,'visible','off')

gray_img = handles.gray_image;
axes(handles.axes2);
imshow(gray_img);
set(handles.axes2text,'String',"Gray Image");
f=fftshift(fft2(gray_img));
fl = log(1+abs(f));
fm = max(fl(:));
axes(handles.axes3);
imshow(im2uint8(fl/fm))
set(handles.axes3text,'String',"Fourier Transform");

function percentageslider_Callback(hObject, eventdata, handles)
global sp_perc
sp_perc = get(hObject,'Value');
text = strcat('Percentage = ',num2str(sp_perc*100));
set(handles.percentagevalue,'String',text);
spbutton_Callback(hObject, eventdata, handles);

function percentageslider_CreateFcn(hObject, eventdata, handles)
global sp_perc
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
sp_perc = get(hObject,'Value');


%--------------------- ADDING SALT AND PEPPER NOISE -----------------------
function spbutton_Callback(hObject, eventdata, handles)
set(handles.sp_panel,'visible','on')
set(handles.sobel_panel,'visible','off')
set(handles.laplace_panel,'visible','off')
set(handles.periodic_panel,'visible','off')
set(handles.kernelvalue,'visible','off')
set(handles.kernelslider,'visible','off')
set(handles.removeperiodicnoisepanel,'visible','off');
set(handles.uipanel20,'visible','off')

global sp_perc 
gray_img = handles.gray_image;
axes(handles.axes2);
imshow(gray_img);
set(handles.axes2text,'String',"Gray Image");
cm_sp = imnoise(gray_img,'salt & pepper',sp_perc);
handles.image_sp = cm_sp;
guidata(hObject,handles);
axes(handles.axes3);
imshow(cm_sp);
set(handles.axes3text,'String',"Salt & Pepper Noise");

function kernelslider_Callback(hObject, eventdata, handles)
global kernel
kernel = get(hObject,'Value');
if (kernel == 0)
    kernel = 1;
end
text = strcat('Kernel Size = ',num2str(kernel));
set(handles.kernelvalue,'String',text);
removespbutton_Callback(hObject, eventdata, handles);

function kernelslider_CreateFcn(hObject, eventdata, handles)
global kernel
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
kernel = get(hObject,'Value');
if (kernel == 0)
    kernel = 1;
end


%------------------- REMOVING SALT AND PEPPER NOISE -----------------------
function removespbutton_Callback(hObject, eventdata, handles)
global kernel sp_perc
opts.WindowStyle = 'non-modal'; opts.Interpreter = 'tex';
if (sp_perc==0)
    err = errordlg('\bf\color{red} THE IMAGE DOES NOT HAVE SALT AND PEPPER NOISE',"ERROR",opts);
else
    set(handles.kernelvalue,'visible','on')
    set(handles.kernelslider,'visible','on')
    cm_sp = handles.image_sp;
    axes(handles.axes2);
    imshow(cm_sp);
    set(handles.axes2text,'String',"Noisy Image");
    cm_sp_med = medfilt2(cm_sp, [kernel,kernel]);
    axes(handles.axes3);
    imshow(cm_sp_med);
    set(handles.axes3text,'String',"Clean Image");
end


%--------------------- ADDING PERIODIC NOISE ------------------------------
function nxslider_Callback(hObject, eventdata, handles)
global nx
nx = get(hObject,'Value');
text = strcat('nx = ',num2str(nx));
set(handles.nxvalue,'String',text);
periodicbutton_Callback(hObject, eventdata, handles);

function nxslider_CreateFcn(hObject, eventdata, handles)
global nx
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
nx = get(hObject,'Value');

function nyslider_Callback(hObject, eventdata, handles)
global ny
ny = get(hObject,'Value');
text = strcat('ny = ',num2str(ny));
set(handles.nyvalue,'String',text);
periodicbutton_Callback(hObject, eventdata, handles);

function nyslider_CreateFcn(hObject, eventdata, handles)
global ny
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
ny = get(hObject,'Value');

function SinCospopupmenu_Callback(hObject, eventdata, handles)
global fn
contents = cellstr(get(hObject,'String'));
popchoice = contents{get(hObject,'Value')};
if(strcmp(popchoice,'Sin'))
    fn=1;
elseif(strcmp(popchoice,'Cos'))
    fn = 2;
end
periodicbutton_Callback(hObject, eventdata, handles);

function SinCospopupmenu_CreateFcn(hObject, eventdata, handles)
global fn
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
fn = 1;

function periodicbutton_Callback(hObject, eventdata, handles)
global nx ny fn pxy
set(handles.periodic_panel,'visible','on')
set(handles.sobel_panel,'visible','off')
set(handles.laplace_panel,'visible','off')
set(handles.sp_panel,'visible','off')
set(handles.removeperiodicnoisepanel,'visible','off');
set(handles.uipanel20,'visible','off')

gray_img = handles.gray_image;
s = size(gray_img);
axes(handles.axes2);
imshow(gray_img);
set(handles.axes2text,'String',"Gray Image");

[x, y]= meshgrid(1:s(2),1:s(1));

Wx = max(max(x));
Wy = max(max(y));
fx = nx/Wx;
fy = ny/Wy;
if (fn == 1)
    pxy = sin(2*pi*fx*x + 2*pi*fy*y)+1; 
elseif (fn == 2)
    pxy = cos(2*pi*fx*x + 2*pi*fy*y)+1; 
end

cp = mat2gray((im2double(gray_img)+pxy));
handles.cp = cp;
guidata(hObject,handles);
axes(handles.axes3);
imshow(cp);
set(handles.axes3text,'String',"Periodic Noise");


%---------------------- REMOVE PERIODIC NOISE -----------------------------
function removeperiodic_popupmenu_Callback(hObject, eventdata, handles)
global method
contents = cellstr(get(hObject,'String'));
popchoice = contents{get(hObject,'Value')};

if(strcmp(popchoice,'Band reject'))
    method=1;
elseif(strcmp(popchoice,'Notch filter'))
    method = 2;
elseif(strcmp(popchoice,'Mask'))
    method = 3;
else
    opts.WindowStyle = 'non-modal'; opts.Interpreter = 'tex';
    err = warndlg('\bf\color{orange} PLEASE SELECT ONE OF THE METHODS',"ERROR",opts);
    set(handles.maskoptions_popupmenu,'visible','off');
    set(handles.applymaskbutton,'visible','off');
    method = 4;
end
removeperiodicnoise_Callback(hObject, eventdata, handles);

function removeperiodic_popupmenu_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function removeperiodicnoise_Callback(hObject, eventdata, handles)
global nx ny method option mask
opts.WindowStyle = 'non-modal'; opts.Interpreter = 'tex';
if (nx ==0 && ny==0)
    err = errordlg('\bf\color{red} THE IMAGE DOES NOT HAVE PERIODIC NOISE',"ERROR",opts);
else
    set(handles.removeperiodicnoisepanel,'visible','on');
    set(handles.periodic_panel,'visible','off')

    gray_img = handles.gray_image;
    s = size(gray_img);

    cp = handles.cp;
    axes(handles.axes2);
    imshow(cp);
    set(handles.axes2text,'String',"Noisy Image");

    cpf=fftshift(fft2(handles.cp));

    [x,y]=meshgrid(-(s(2)/2):(s(2)/2)-1,-(s(1)/2):(s(1)/2)-1);
    z=sqrt(x.^2+y.^2);
    z((((s(1)/2)+1+ny)),(((s(2)/2)+1+nx)));
    z((((s(1)/2)+1-ny)),(((s(2)/2)+1-nx)));
    distance = z((s(1)/2)+1+ny,(s(2)/2)+1+nx);

    if(method == 1)
        set(handles.maskoptions_popupmenu,'visible','off');
        set(handles.applymaskbutton,'visible','off');

        br = (z < (distance -2) | z > (distance +2));
        cpfbr=cpf.*br;
        axes(handles.axes3);
        fftshow((ifft2(cpfbr)));
        set(handles.axes3text,'String',"Clean Image by Band reject");

    elseif(method == 2)
        set(handles.maskoptions_popupmenu,'visible','off');
        set(handles.applymaskbutton,'visible','off');
        
        opts.WindowStyle = 'non-modal'; opts.Interpreter = 'tex';
        if (nx == 0)
            cpf(s(1)/2+1+ny,:)=0;
            cpf(s(1)/2+1-ny,:)=0;
        elseif(ny == 0)
            cpf(:,s(2)/2+1+nx)=0;
            cpf(:,s(2)/2+1-nx)=0;
        else
            cpf(s(1)/2+1+ny,:)=0;
            cpf(s(1)/2+1-ny,:)=0;
            cpf(:,s(2)/2+1+nx)=0;
            cpf(:,s(2)/2+1-nx)=0;
        end
        axes(handles.axes3);
        fftshow((ifft2(cpf)));
        set(handles.axes3text,'String',"Clean Image by Notch filter");

    elseif(method == 3)
        set(handles.maskoptions_popupmenu,'visible','on');

        cm = gray_img;
        cp = handles.cp;
        freqImageOriginal = fftshift(fft2(cm));
        magImageOriginal = log(abs(freqImageOriginal));
        freqImageNoisy = fftshift(fft2(cp));
        magImageNoisy = log(abs(freqImageNoisy));

        mask = ones(size(magImageNoisy));

        if(option == 1)
            set(handles.applymaskbutton,'visible','off');
            set(handles.uipanel20,'visible','off')
            set(handles.uipanel10,'visible','on')
            set(handles.uipanel11,'visible','on')
            set(handles.uipanel12,'visible','on')
            set(handles.axes1text,'visible','on')
            set(handles.axes2text,'visible','on')
            set(handles.axes3text,'visible','on')
            mask(s(1)/2+1+ny,s(2)/2+1+nx) = 0;
            mask(s(1)/2+1-ny,s(2)/2+1-nx) = 0;

            filtered = mask.*freqImageNoisy;
            amplitudeImage = log(abs(filtered));
            filteredImage = ifft2(ifftshift(filtered));
            ampFilteredImage = abs(filteredImage);
            minValue = min(min(ampFilteredImage));
            maxValue = max(max(ampFilteredImage));

            axes(handles.axes3);
            imshow(ampFilteredImage, [minValue maxValue]);
            set(handles.axes3text,'String',"Clean Image by Default Mask");

            elseif(option == 2)
            set(handles.applymaskbutton,'visible','on');
            opts.WindowStyle = 'non-modal'; opts.Interpreter = 'tex';
            hint = msgbox('\bf\color{lightBlue} Press on "Apply Mask" button, Zoom in on desired region then press spacebar to Choose points', 'Hint','help',opts);               
        end
    elseif(method == 4)
        return;
    end
end


%-------------------------------- MASK ------------------------------------
function maskoptions_popupmenu_Callback(hObject, eventdata, handles)
global option
contents = cellstr(get(hObject,'String'));
popchoice = contents{get(hObject,'Value')};

if(strcmp(popchoice,'Default Mask'))
    option=1;
elseif(strcmp(popchoice,'Custom Mask'))
    option = 2;
end
removeperiodicnoise_Callback(hObject, eventdata, handles);

function maskoptions_popupmenu_CreateFcn(hObject, eventdata, handles)
global option
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
option=1;

function applymaskbutton_Callback(hObject, eventdata, handles)
set(handles.uipanel20,'visible','on')
set(handles.uipanel10,'visible','off')
set(handles.uipanel11,'visible','off')
set(handles.uipanel12,'visible','off')
set(handles.axes1text,'visible','off')
set(handles.axes2text,'visible','off')
set(handles.axes3text,'visible','off')

gray_img = handles.gray_image;
cm = gray_img;
cp = handles.cp;
freqImageOriginal = fftshift(fft2(cm));
magImageOriginal = log(abs(freqImageOriginal));
freqImageNoisy = fftshift(fft2(cp));
magImageNoisy = log(abs(freqImageNoisy));

mask = ones(size(magImageNoisy));

f=fftshift(fft2(cp));
fl = log(1+abs(f));
fm = max(fl(:));
axes(handles.axes5);
imshow(im2uint8(fl/fm));

zoom on;
pause();
zoom off; 
axes(handles.axes5),
[x_input,y_input] = ginput(2);
zoom out;

x_pt = x_input;
y_pt = y_input;
x1 = round(x_pt(1));
y1 = round(y_pt(1));
x2 = round(x_pt(2));
y2 = round(y_pt(2));

mask(y1,x1) = 0;
mask(y2,x2) = 0;

set(handles.uipanel20,'visible','off')
set(handles.uipanel10,'visible','on')
set(handles.uipanel11,'visible','on')
set(handles.uipanel12,'visible','on')
set(handles.axes1text,'visible','on')
set(handles.axes2text,'visible','on')
set(handles.axes3text,'visible','on')

filtered = mask.*freqImageNoisy;
amplitudeImage = log(abs(filtered));
filteredImage = ifft2(ifftshift(filtered));
ampFilteredImage = abs(filteredImage);
minValue = min(min(ampFilteredImage));
maxValue = max(max(ampFilteredImage));

axes(handles.axes3);
imshow(ampFilteredImage, [minValue maxValue]);
set(handles.axes3text,'String',"Clean Image by Custom Mask");


%---------------------------- FUNCTIONS USED ------------------------------
function fftshow(f,type)
if nargin<2
type='log';
end
if (type=='log')
fl = log(1+abs(f));
fm = max(fl(:));
imshow(im2uint8(fl/fm))
elseif (type=='abs')
fa=abs(f);
fm=max(fa(:));
imshow(fa/fm)
else
error('TYPE must be abs or log.');
end
