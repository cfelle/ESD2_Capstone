function Lab4
%% Lab 4 - Images from Blender (Stereo Ball Distance)
% Author - Cooper Felle
%
% PURPOSE (per Lab 4 PDF):
%   - Connect MatLab to Blender and render left/right camera images.
%   - Detect the ball centroid in BOTH images.
%   - Use stereo triangulation to compute the 3D position (X,Y,Z) of the ball.
%   - Benchmark/plot distance vs accuracy for distances from 3 m to 5 m
%     with the ball centered between the cameras.

clc; close all;

%% -------------------- USER SETTINGS --------------------
% Blender server settings
BLENDER.server_ip   = '127.0.0.1';
BLENDER.server_port = 55001;

% Object names in Blender
BLENDER.ballName   = "Ball";     % must match Blender Outliner name
BLENDER.cameraName = "Camera";   % must match Blender Outliner name

% Camera BASE pose (absolute) in Blender world coordinates.
% IMPORTANT: blenderServer.py sets obj.location ABSOLUTE, not offset.
% So we must send the camera's real base location here, then add +/-B/2.
% Set these to match your Blender Camera Transform panel.
BLENDER.camBaseX = 0.0;
BLENDER.camBaseY = 0.0;
BLENDER.camBaseZ = 10.0;

% Render size (must match what you expect in MATLAB)
BLENDER.width  = 752;
BLENDER.height = 480;

% Camera orientation to send (degrees). In patched blenderServer.py, if all
% are 0, the server preserves the original camera rotation.
BLENDER.camPitch = 0;
BLENDER.camRoll  = 0;
BLENDER.camYaw   = 0;

% Ball orientation to send (degrees) - not needed for tracking, but kept.
BLENDER.ballPitch = 0;
BLENDER.ballRoll  = 0;
BLENDER.ballYaw   = 0;

% ---------- Z (ground-truth / benchmark) ----------
% In this version we render ONE stereo pair at a time from Blender.
% For benchmarking (distance vs accuracy), we sweep Ztrue from 3..5 m.
defaults.Ztrue_m = 3.0;

% use f_px directly (pixels) and baseline B (meters)
defaults.f_px = 800;     % focal length in pixels (set to match your Blender camera intrinsics)
defaults.B_m  = 0.10;    % baseline in meters (camera separation)

% principal point defaults (auto set to image center on render)
defaults.cx   = 376;     % will be overwritten by actual image width/2
defaults.cy   = 240;     % overwritten by image height/2

% detection defaults
defaults.method = "GrayThresh";   % GrayThresh | YCbCrNeutral | Circles
defaults.grayThresh = 0.00;       % for GrayThresh (0 = auto/Otsu)
defaults.minBlobArea = 800;
defaults.maxBlobArea = 2e4;

% YCbCr neutral thresholds
defaults.yMin = 0.45;            % brightness threshold for YCbCr (0..1)
defaults.tCbCr = 0.15;           % tolerance around neutral chroma (0..0.5)

defaults.useTopCrop = false;      % optional: crop to top fraction
defaults.topFrac    = 0.75;

% overlay
defaults.overlayRadiusPx = 16;

%% -------------------- GUI SETUP --------------------
ui = uifigure('Name','Lab 4 - Stereo Ball Localization (Blender)','Position',[100 100 1200 700]);

client = [];

busy = false;                 % prevents overlapping Blender calls
autoTimer = timer( ...
    'ExecutionMode','singleShot', ...
    'StartDelay', 0.15, ...    % debounce delay (seconds)
    'TimerFcn', @(~,~) safeUpdateOnce() ); %#ok<NASGU>

ui.CloseRequestFcn = @(~,~) onClose();

function onClose()
    try
        if ~isempty(client) && isvalid(client)
            clear client;
        end
    catch
    end
    delete(ui);
end

main = uigridlayout(ui,[3 3]);
main.ColumnWidth = {430,'1x','1x'};
main.RowHeight   = {190, 300, '1x'};
main.Padding = [10 10 10 10];
main.RowSpacing = 10;
main.ColumnSpacing = 10;

% Panels for left and right images
pL = uipanel(main,'Title','Left Image');
pL.Layout.Row = 1; pL.Layout.Column = 2;
axL = uiaxes(pL);
axL.Position = [10 10 pL.Position(3)-20 pL.Position(4)-40];
axis(axL,'image'); axis(axL,'off');

pR = uipanel(main,'Title','Right Image');
pR.Layout.Row = 1; pR.Layout.Column = 3;
axR = uiaxes(pR);
axR.Position = [10 10 pR.Position(3)-20 pR.Position(4)-40];
axis(axR,'image'); axis(axR,'off');

% 3D plot panel
p3 = uipanel(main,'Title','3D Ball Position');
p3.Layout.Row = [2 3]; 
p3.Layout.Column = [2 3];

g3 = uigridlayout(p3,[1 1]);
g3.Padding = [6 6 6 6];   % small border
ax3 = uiaxes(g3);
grid(ax3,'on'); view(ax3,3);
xlabel(ax3,'X (m)'); ylabel(ax3,'Y (m)'); zlabel(ax3,'Z (m)');
title(ax3,'3D Ball Position');

% Controls (top-left)
pCtrl = uipanel(main,'Title','Controls');
pCtrl.Layout.Row = 1;
pCtrl.Layout.Column = 1;

g = uigridlayout(pCtrl,[10 3]);
g.RowHeight = {36,28,28,36,36,28,28,28,28,34};
g.ColumnWidth = {160,'1x',70};
g.Padding = [10 10 10 10];
g.RowSpacing = 8;
g.ColumnSpacing = 8;

% Row 1: Ztrue (meters)
uilabel(g,'Text','Z true (m)');
sZ = uislider(g,'Limits',[3 5],'Value',defaults.Ztrue_m);
sZ.Layout.Row = 1; sZ.Layout.Column = 2;
eZ = uieditfield(g,'numeric','Value',defaults.Ztrue_m);
eZ.Layout.Row = 1; eZ.Layout.Column = 3;

% Row 2: method
uilabel(g,'Text','Detection method');
ddMethod = uidropdown(g,'Items',["GrayThresh","YCbCrNeutral","Circles"],'Value',defaults.method);
ddMethod.Layout.Row = 2; ddMethod.Layout.Column = [2 3];

% Row 3: f_px
uilabel(g,'Text','f (pixels)');
eF = uieditfield(g,'numeric','Value',defaults.f_px);
eF.Layout.Row = 3; eF.Layout.Column = [2 3];

% Row 4: baseline B
uilabel(g,'Text','Baseline B (m)');
eB = uieditfield(g,'numeric','Value',defaults.B_m);
eB.Layout.Row = 4; eB.Layout.Column = [2 3];

% Row 5: gray thresh
uilabel(g,'Text','Gray thresh (0..1)');
sGray = uislider(g,'Limits',[0 1],'Value',defaults.grayThresh);
sGray.Layout.Row = 5; sGray.Layout.Column = 2;
eGray = uieditfield(g,'numeric','Value',defaults.grayThresh,'Limits',[0 1]);
eGray.Layout.Row = 5; eGray.Layout.Column = 3;

% Row 6: min blob area
uilabel(g,'Text','Min blob area');
eMinA = uieditfield(g,'numeric','Value',defaults.minBlobArea);
eMinA.Layout.Row = 6; eMinA.Layout.Column = [2 3];

% Row 7: max blob area
uilabel(g,'Text','Max blob area');
eMaxA = uieditfield(g,'numeric','Value',defaults.maxBlobArea);
eMaxA.Layout.Row = 7; eMaxA.Layout.Column = [2 3];

% Row 8: YCbCr brightness min (only used for YCbCrNeutral)
uilabel(g,'Text','YCbCr: Y min (0=auto)');
eYmin = uieditfield(g,'numeric','Value',0,'Limits',[0 1]);  % Value=0 means AUTO
eYmin.Layout.Row = 8; eYmin.Layout.Column = [2 3];

% Row 9: YCbCr chroma tolerance (only used for YCbCrNeutral)
uilabel(g,'Text','YCbCr: chroma tol');
eTcbcr = uieditfield(g,'numeric','Value',defaults.tCbCr,'Limits',[0 0.5]);
eTcbcr.Layout.Row = 9; eTcbcr.Layout.Column = [2 3];

% Row 10: top crop checkbox + frac
cbTop = uicheckbox(g,'Text','Search only top of image','Value',defaults.useTopCrop);
cbTop.Layout.Row = 10; cbTop.Layout.Column = [1 2];
eTopFrac = uieditfield(g,'numeric','Value',defaults.topFrac,'Limits',[0.1 1]);
eTopFrac.Layout.Row = 10; eTopFrac.Layout.Column = 3;

% Output panel (bottom-left)
pOut = uipanel(main,'Title','Output');
pOut.Layout.Row = [2 3];
pOut.Layout.Column = 1;

go = uigridlayout(pOut,[8 1]);
go.RowHeight = {40,24,24,24,24,24,24,'1x'};
go.Padding = [10 10 10 10];
go.RowSpacing = 8;

btnCalc = uibutton(go,'Text','Calculate','ButtonPushedFcn',@(~,~)updateOnce());
btnCalc.Layout.Row = 1;

lblL = uilabel(go,'Text','Left centroid (u,v): --');  lblL.Layout.Row = 2;
lblR = uilabel(go,'Text','Right centroid (u,v): --'); lblR.Layout.Row = 3;
lblD = uilabel(go,'Text','Disparity d (px): --');     lblD.Layout.Row = 4;
lblX = uilabel(go,'Text','X (m): --');                lblX.Layout.Row = 5;
lblY = uilabel(go,'Text','Y (m): --');                lblY.Layout.Row = 6;
lblZ = uilabel(go,'Text','Z (m): --');                lblZ.Layout.Row = 7;

msg = uitextarea(go,'Editable','off');
msg.Layout.Row = 8;

% Bindings between slider and edit box (Ztrue)
sZ.ValueChangingFcn = @(src,evt)set(eZ,'Value',evt.Value);
eZ.ValueChangedFcn  = @(src,evt)set(sZ,'Value',src.Value);

% Gray thresh link
sGray.ValueChangingFcn = @(src,evt)set(eGray,'Value',evt.Value);
eGray.ValueChangedFcn  = @(src,evt)set(sGray,'Value',src.Value);

%% -------------------- TCP CLIENT STATE --------------------
client = [];

%% -------------------- CORE CALLBACK --------------------
    function updateOnce()
        % Prevent overlapping calls (overlap can corrupt TCP stream and crash the Blender timer)
        if busy
            return;
        end
        busy = true;
        btnCalc.Enable = 'off';
        cleanupObj = onCleanup(@() setBusyFalse()); %#ok<NASGU>

        % read params from UI
        params = defaults;
        params.Ztrue_m     = eZ.Value;
        params.method      = string(ddMethod.Value);
        params.f_px        = eF.Value;
        params.B_m         = eB.Value;
        params.grayThresh  = eGray.Value;
        params.minBlobArea = eMinA.Value;
        params.maxBlobArea = eMaxA.Value;
        params.yMin        = eYmin.Value;
        params.tCbCr       = eTcbcr.Value;
        params.useTopCrop  = cbTop.Value;
        params.topFrac     = eTopFrac.Value;

        Ztrue = params.Ztrue_m;

        disp("=== updateOnce() CALLED ===");
        fprintf("Ztrue=%.2f m  method=%s\n", Ztrue, params.method);
        drawnow;

        % ---- Stage 1: Get images from Blender ----
        tIO = tic;
        disp("Calling getStereoFromBlender...");
        [IL, IR, infoLines] = getStereoFromBlender(Ztrue, params);

        % ---- DEBUG: did we actually get images? ----
        fprintf("DEBUG: isempty(IL)=%d, isempty(IR)=%d\n", isempty(IL), isempty(IR));
        if ~isempty(IL)
            fprintf("DEBUG: IL size = [%d %d %d], class=%s, min=%d, max=%d\n", ...
                size(IL,1), size(IL,2), size(IL,3), class(IL), min(IL(:)), max(IL(:)));
        end
        if ~isempty(IR)
            fprintf("DEBUG: IR size = [%d %d %d], class=%s, min=%d, max=%d\n", ...
                size(IR,1), size(IR,2), size(IR,3), class(IR), min(IR(:)), max(IR(:)));
        end

        disp("Returned from getStereoFromBlender.");
        msIO = 1000*toc(tIO);
        fprintf("msIO = %.1f ms\n", msIO);

        % show info lines
        if ~isempty(infoLines)
            msg.Value = infoLines;
        else
            msg.Value = "No info lines.";
        end

        if isempty(IL) || isempty(IR)
            % show blank axes
            cla(axL); cla(axR); cla(ax3);
            lblL.Text = "Left centroid (u,v): --";
            lblR.Text = "Right centroid (u,v): --";
            lblD.Text = "Disparity d (px): --";
            lblX.Text = "X (m): --";
            lblY.Text = "Y (m): --";
            lblZ.Text = "Z (m): --";
            return;
        end

        % display the images
        imshow(IL,'Parent',axL);
        title(axL, sprintf("Left (Z=%.1fm)", Ztrue));

        imshow(IR,'Parent',axR);
        title(axR, sprintf("Right (Z=%.1fm)", Ztrue));

        % ---- Stage 2: Detect centroids ----
        [cL, dbgL] = detectBallCentroid(IL, params);
        [cR, dbgR] = detectBallCentroid(IR, params);

        if any(isnan(cL)) || any(isnan(cR))
            msg.Value = [
                infoLines(:)
                "Centroid detection failed."
                "Left dbg:"
                dbgL(:)
                "Right dbg:"
                dbgR(:)
            ];

            return;
        end

        % overlay centroid markers (point + circle so it's obvious)
        rPx = max(6, round(params.overlayRadiusPx));
        hold(axL,'on');
        plot(axL, cL(1), cL(2), 'g+','MarkerSize',10,'LineWidth',2);
        try, viscircles(axL, cL, rPx, 'Color','g','LineWidth',1); catch, end
        hold(axL,'off');

        hold(axR,'on');
        plot(axR, cR(1), cR(2), 'g+','MarkerSize',10,'LineWidth',2);
        try, viscircles(axR, cR, rPx, 'Color','g','LineWidth',1); catch, end
        hold(axR,'off');

        lblL.Text = sprintf("Left centroid (u,v): (%.1f, %.1f)", cL(1), cL(2));
        lblR.Text = sprintf("Right centroid (u,v): (%.1f, %.1f)", cR(1), cR(2));

        % ---- Stage 3: Stereo triangulation ----
        % principal point from image center
        w = size(IL,2);
        h = size(IL,1);
        cx = w/2;
        cy = h/2;

        f = params.f_px;
        B = params.B_m;

        d = (cL(1) - cR(1)); % disparity in pixels
        lblD.Text = sprintf("Disparity d (px): %.2f", d);

        if abs(d) < 1e-6
            msg.Value = [
                infoLines
                "Disparity ~ 0 -> cannot triangulate (infinite Z)."
            ];
            return;
        end

        Z = (f * B) / d;
        X = ((cL(1) - cx) * Z) / f;
        Y = ((cL(2) - cy) * Z) / f;

        lblX.Text = sprintf("X (m): %.3f", X);
        lblY.Text = sprintf("Y (m): %.3f", Y);
        lblZ.Text = sprintf("Z (m): %.3f", Z);

        % plot in 3D
        cla(ax3);
        plot3(ax3, X, Y, Z, 'ro','MarkerSize',10,'LineWidth',2);
        grid(ax3,'on'); view(ax3,3);
        xlabel(ax3,'X (m)'); ylabel(ax3,'Y (m)'); zlabel(ax3,'Z (m)');
        title(ax3,'3D Ball Position');
        
        % Auto limits so the ball is always visible
        padXY = 0.25;              % meters
        padZ  = max(0.5, 0.2*Z);   % scale with distance
        
        xlim(ax3, [X - padXY, X + padXY]);
        ylim(ax3, [Y - padXY, Y + padXY]);
        zlim(ax3, [max(0, Z - padZ), Z + padZ]);

        msg.Value = [
            infoLines
            "Centroids OK."
            sprintf("X=%.3f  Y=%.3f  Z=%.3f", X, Y, Z)
        ];
    end

    function setBusyFalse()
        busy = false;
        if isvalid(btnCalc)
            btnCalc.Enable = 'on';
        end
    end

%% -------------------- BLENDER I/O --------------------
    function [IL, IR, infoLines] = getStereoFromBlender(Ztrue_m, params)
        % Render left/right images from Blender for a ball at (0,0,Ztrue_m)
        % and cameras separated by baseline B along X.
        %
        % NOTE:
        %   Blender scene contains a plane (the “square”). If the ball is placed
        %   coplanar/inside that plane, it will be occluded. We add a small Z offset
        %   toward the camera to ensure the ball is visible.

        infoLines = string.empty(0,1);
        IL = [];
        IR = [];

        % Ensure a client exists (reconnect only here)
        if ~ensureClient()
            infoLines = ["No TCP client (ensureClient failed)."];
            return;
        end

        % Lock the current client for this whole request
        c = client;

        % Define camera translations (centered baseline)
        B = params.B_m;
        camXL = -B/2;
        camXR = +B/2;

        % UI update
        msg.Value = [
            sprintf("Rendering from Blender... Ztrue = %.2f m", Ztrue_m)
            sprintf("Ball=%s | Camera=%s | B=%.3f", BLENDER.ballName, BLENDER.cameraName, B)
        ];
        drawnow;

        % -----------------------------------------------------------------
        % STEP 1) Move the ball (ground-truth placement)
        % -----------------------------------------------------------------
        zOffset_m = 0.25;                 % meters toward camera (adjust if needed)
        zBall     = -Ztrue_m + zOffset_m; % final Z sent to Blender

        try
            disp("STEP 1/3: move ball");
            drawnow;

            blenderLink(c, BLENDER.width, BLENDER.height, ...
                0, 0, zBall, ...
                BLENDER.ballPitch, BLENDER.ballRoll, BLENDER.ballYaw, ...
                BLENDER.ballName);

            disp("STEP 1 DONE");
        catch ME
            infoLines = [
                "Error moving/reading Ball from Blender."
                "Check BLENDER.ballName matches the ball object name in Blender."
                " "
                string(ME.message)
            ];
            return;
        end

        % -----------------------------------------------------------------
        % STEP 2) Render LEFT (camera offset by -B/2 along X)
        % NOTE: blenderServer.py sets camera location ABSOLUTE, so we send (camBaseX+camXL, camBaseY, camBaseZ).
        % -----------------------------------------------------------------
        try
            disp("STEP 2/3: render LEFT");
            drawnow;

            IL = blenderLink(c, BLENDER.width, BLENDER.height, ...
                BLENDER.camBaseX+camXL, BLENDER.camBaseY, BLENDER.camBaseZ, ...
                BLENDER.camPitch, BLENDER.camRoll, BLENDER.camYaw, ...
                BLENDER.cameraName);

            disp("STEP 2 DONE");
        catch ME
            infoLines = [
                "Error rendering Left image from Blender."
                "Check BLENDER.cameraName matches the camera object name in Blender."
                " "
                string(ME.message)
            ];
            IL = [];
            return;
        end

        % -----------------------------------------------------------------
        % STEP 3) Render RIGHT (camera offset by +B/2 along X)
        % -----------------------------------------------------------------
        try
            disp("STEP 3/3: render RIGHT");
            drawnow;

            IR = blenderLink(c, BLENDER.width, BLENDER.height, ...
                BLENDER.camBaseX+camXR, BLENDER.camBaseY, BLENDER.camBaseZ, ...
                BLENDER.camPitch, BLENDER.camRoll, BLENDER.camYaw, ...
                BLENDER.cameraName);

            disp("STEP 3 DONE");
        catch ME
            infoLines = [
                "Error rendering Right image from Blender."
                "Check BLENDER.cameraName matches the camera object name in Blender."
                " "
                string(ME.message)
            ];
            IR = [];
            return;
        end

        infoLines = [
            "Blender renders OK."
            sprintf("Ball set to (0,0,%.2f)  [Ztrue=%.2f, zOffset=%.2f]", 0+zBall, Ztrue_m, zOffset_m)
            sprintf("CamL=(%.3f,%.3f,%.3f)  CamR=(%.3f,%.3f,%.3f)", BLENDER.camBaseX+camXL, BLENDER.camBaseY, BLENDER.camBaseZ, BLENDER.camBaseX+camXR, BLENDER.camBaseY, BLENDER.camBaseZ)
        ];
    end

%% -------------------- DETECTION --------------------
    function [centroid, dbg] = detectBallCentroid(I, params)
        dbg = string.empty(0,1);
        centroid = [NaN NaN];

        if isempty(I)
            dbg = ["Empty image."];
            return;
        end

        % optional crop to top region
        if params.useTopCrop
            H = size(I,1);
            Hc = max(1, round(params.topFrac * H));
            Iwork = I(1:Hc,:,:);
        else
            Iwork = I;
        end

        method = params.method;

        switch method
            case "GrayThresh"
                Ig = rgb2gray(Iwork);
                Igd = im2double(Ig);

                if params.grayThresh <= 0
                    % Auto threshold (Otsu). Works well across different lighting/exposure.
                    t = graythresh(Ig);
                else
                    t = params.grayThresh;
                end

                BW = Igd > t;
                dbg(end+1) = sprintf("GrayThresh: t=%.3f  Ig[min,max]=[%.3f, %.3f]", t, min(Igd(:)), max(Igd(:)));

            case "YCbCrNeutral"
                YCbCr = rgb2ycbcr(Iwork);
                Cb = im2double(YCbCr(:,:,2));
                Cr = im2double(YCbCr(:,:,3));
                Y  = im2double(YCbCr(:,:,1));
            
                t = params.tCbCr;
            
                % AUTO brightness threshold if yMin <= 0
                if params.yMin <= 0
                    yMin = prctile(Y(:), 97);      % focuses on brightest stuff (ball highlight)
                else
                    yMin = params.yMin;
                end
            
                BW = (abs(Cb - 0.5) < t) & (abs(Cr - 0.5) < t) & (Y > yMin);
            
                dbg(end+1) = sprintf("YCbCr: yMin=%.3f t=%.2f  Y[min,max]=[%.2f,%.2f]", ...
                                     yMin, t, min(Y(:)), max(Y(:)));

            case "Circles"
                Ig = rgb2gray(Iwork);
                % Hough circle search (tune these if needed)
                [centers, radii] = imfindcircles(Ig,[10 200],'ObjectPolarity','bright','Sensitivity',0.92); %#ok<ASGLU>
                if isempty(centers)
                    dbg = ["imfindcircles found nothing."];
                    return;
                end
                % take strongest circle
                centroid = centers(1,:);
                return;

            otherwise
                dbg = ["Unknown method."];
                return;
        end

        % quick sanity info
        dbg(end+1) = sprintf("BW white pixels = %d", nnz(BW));

        % blob filter
        BW = bwareaopen(BW, max(1, round(params.minBlobArea)));
        stats = regionprops(BW,'Area','Centroid');

        if isempty(stats)
            dbg(end+1) = "No blobs after threshold/area filter.";
            return;
        end

        areas = [stats.Area];
        ok = (areas >= params.minBlobArea) & (areas <= params.maxBlobArea);

        stats = stats(ok);
        if isempty(stats)
            dbg(end+1) = "Blobs found, but none within [min,max] area.";
            return;
        end

        % choose largest remaining blob
        [~,k] = max([stats.Area]);
        c = stats(k).Centroid;

        centroid = [c(1), c(2)];
    end

%% -------------------- TCP HELPERS --------------------
    function ok = ensureClient()
        ok = true;
        try
            if isempty(client) || ~isvalid(client)
                disp("Connecting to Blender at 127.0.0.1:55001");
                client = tcpclient(BLENDER.server_ip, BLENDER.server_port, 'Timeout', 20);
                % NOTE: blenderServer.py accepts only ONE connection per 'Start Server'.
                % If MATLAB ever drops the socket, you'll need to press Stop/Start in Blender before reconnecting.
            end
        catch ME
            ok = false;
            msg.Value = [
                "Failed to create tcpclient."
                string(ME.message)
                "Make sure Blender server is running (Start Server)."
            ];
        end
    end

    function safeUpdateOnce()
        % (Optional) helper for debounced updates if you later want auto-updating controls
        try
            updateOnce();
        catch ME
            msg.Value = [
                "safeUpdateOnce error:"
                string(ME.message)
            ];
        end
    end
end
