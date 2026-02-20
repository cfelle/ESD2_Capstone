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
defaults.f_px = 2632;     % focal length in pixels (set to match your Blender camera intrinsics)
defaults.B_m  = 0.10;     % baseline in meters (camera separation)
defaults.camZ = 10.0;     % Blender camera world Z (matches CamL/CamR z used for rendering)
defaults.Zoffset = 0.25;  % Ball is set to z = Ztrue - Zoffset in Blender (matches your server move)

% principal point defaults (auto set to image center on render)
defaults.cx   = 376;      % overwritten by actual image width/2
defaults.cy   = 240;      % overwritten by actual image height/2

% detection defaults
defaults.method = "GrayThresh";   % GrayThresh | YCbCrNeutral | Circles
defaults.grayThresh = 0.00;        % for GrayThresh (0 = auto/Otsu)
defaults.minBlobArea = 100;
defaults.maxBlobArea = 2e4;

% YCbCr neutral thresholds
defaults.yMin = 0.45;      % brightness threshold for YCbCr (0..1)
defaults.tCbCr = 0.15;     % tolerance around neutral chroma (0..0.5)

defaults.useTopCrop = false;  % optional: crop to top fraction
defaults.topFrac    = 0.75;

% overlay
defaults.overlayRadiusPx = 16;

%% -------------------- GUI SETUP --------------------
ui = uifigure('Name','Lab 4 - Stereo Ball Localization (Blender)','Position',[100 100 1200 700]);
ui.CloseRequestFcn = @(~,~) onClose();

client = [];

busy = false;                 % prevents overlapping Blender calls
pendingUpdate = false;        % UI changed while busy -> run one more update

% Keep last-good centroids to stabilize tracking and prevent L/R flips
lastCL = [];
lastCR = [];

autoTimer = timer( ...
    'ExecutionMode','singleShot', ...
    'StartDelay', 0.15, ...    % debounce delay (seconds)
    'TimerFcn', @(~,~) safeUpdateOnce() ); %#ok<NASGU>


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

% Persistent 3D point handle (IMPORTANT: prevents "stale point" when we skip updates)
hPt3 = plot3(ax3, NaN, NaN, NaN, 'ro', 'MarkerSize', 10, 'LineWidth', 2);

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
        sZ.ValueChangedFcn = @(src,~) onZSlider(src);
sZ.Layout.Row = 1; sZ.Layout.Column = 2;
eZ = uieditfield(g,'numeric','Value',defaults.Ztrue_m);
        eZ.ValueChangedFcn = @(src,~) onEditZ(src);
eZ.Layout.Row = 1; eZ.Layout.Column = 3;

% Row 2: method
uilabel(g,'Text','Detection method');
ddMethod = uidropdown(g,'Items',["GrayThresh","YCbCrNeutral","Circles"],'Value',defaults.method);
        ddMethod.ValueChangedFcn = @(~,~) onParamChange();
ddMethod.Layout.Row = 2; ddMethod.Layout.Column = [2 3];

% Row 3: f_px
uilabel(g,'Text','f (pixels)');
eF = uieditfield(g,'numeric','Value',defaults.f_px);
        eF.ValueChangedFcn = @(~,~) onParamChange();
eF.Layout.Row = 3; eF.Layout.Column = [2 3];

% Row 4: baseline B
uilabel(g,'Text','Baseline B (m)');
eB = uieditfield(g,'numeric','Value',defaults.B_m);
        eB.ValueChangedFcn = @(~,~) onParamChange();
eB.Layout.Row = 4; eB.Layout.Column = [2 3];

% Row 5: gray thresh
uilabel(g,'Text','Gray thresh (0..1)');
sGray = uislider(g,'Limits',[0 1],'Value',defaults.grayThresh);
        sGray.ValueChangedFcn = @(src,~) onGraySlider(src);
sGray.Layout.Row = 5; sGray.Layout.Column = 2;
eGray = uieditfield(g,'numeric','Value',defaults.grayThresh,'Limits',[0 1]);
        eGray.ValueChangedFcn = @(src,~) onGrayEdit(src);
eGray.Layout.Row = 5; eGray.Layout.Column = 3;

% Row 6: min blob area
uilabel(g,'Text','Min blob area');
eMinA = uieditfield(g,'numeric','Value',defaults.minBlobArea);
        eMinA.ValueChangedFcn = @(~,~) onParamChange();
eMinA.Layout.Row = 6; eMinA.Layout.Column = [2 3];

% Row 7: max blob area
uilabel(g,'Text','Max blob area');
eMaxA = uieditfield(g,'numeric','Value',defaults.maxBlobArea);
        eMaxA.ValueChangedFcn = @(~,~) onParamChange();
eMaxA.Layout.Row = 7; eMaxA.Layout.Column = [2 3];

% Row 8: YCbCr brightness min (only used for YCbCrNeutral)
uilabel(g,'Text','YCbCr: Y min (0=auto)');
eYmin = uieditfield(g,'numeric','Value',0,'Limits',[0 1]);  % Value=0 means AUTO
        eYmin.ValueChangedFcn = @(~,~) onParamChange();
eYmin.Layout.Row = 8; eYmin.Layout.Column = [2 3];

% Row 9: YCbCr chroma tolerance (only used for YCbCrNeutral)
uilabel(g,'Text','YCbCr: chroma tol');
eTcbcr = uieditfield(g,'numeric','Value',defaults.tCbCr,'Limits',[0 0.5]);
        eTcbcr.ValueChangedFcn = @(~,~) onParamChange();
eTcbcr.Layout.Row = 9; eTcbcr.Layout.Column = [2 3];

% Row 10: top crop checkbox + frac
cbTop = uicheckbox(g,'Text','Search only top of image','Value',defaults.useTopCrop);
        cbTop.ValueChangedFcn = @(~,~) onParamChange();
cbTop.Layout.Row = 10; cbTop.Layout.Column = [1 2];
eTopFrac = uieditfield(g,'numeric','Value',defaults.topFrac,'Limits',[0.1 1]);
        eTopFrac.ValueChangedFcn = @(~,~) onParamChange();
eTopFrac.Layout.Row = 10; eTopFrac.Layout.Column = 3;

% Output panel (bottom-left)
pOut = uipanel(main,'Title','Output');
pOut.Layout.Row = [2 3];
pOut.Layout.Column = 1;

go = uigridlayout(pOut,[8 1]);
go.RowHeight = {40,24,24,24,24,24,24,'1x'};
go.Padding = [10 10 10 10];
go.RowSpacing = 8;

btnCalc = uibutton(go,'Text','Calculate','ButtonPushedFcn',@(~,~) safeUpdateOnce());
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

% Gray thresh link
sGray.ValueChangingFcn = @(src,evt)set(eGray,'Value',evt.Value);

%% -------------------- TCP CLIENT STATE --------------------
client = [];

%% -------------------- CORE CALLBACK --------------------
    function updateOnce()
        % Prevent overlapping calls (overlap can corrupt TCP stream and crash the Blender timer)
        if busy
            pendingUpdate = true;
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
            % show blank axes and CLEAR 3D point
            cla(axL); cla(axR);
            hPt3.XData = NaN; hPt3.YData = NaN; hPt3.ZData = NaN;

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
        paramsL = params; paramsR = params;
        paramsL.refCentroid = lastCL;
        paramsR.refCentroid = lastCR;

        [cL, dbgL] = detectBallCentroid(IL, paramsL);
        [cR, dbgR] = detectBallCentroid(IR, paramsR);

        if any(isnan(cL)) || any(isnan(cR))
            % IMPORTANT: clear 3D point when we skip update
            hPt3.XData = NaN; hPt3.YData = NaN; hPt3.ZData = NaN;

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

        % update last-good centroids (stabilizes tracking and helps Circles choice)
        lastCL = cL;
        lastCR = cR;

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
        shift_x = 0.25;
        shift_y = 0.0;

        cx = w/2  - shift_x * w;
        cy = h/2 + shift_y * h;

        f = params.f_px;
        B = params.B_m;

        d = (cL(1) - cR(1)); % disparity in pixels
        lblD.Text = sprintf("Disparity d (px): %.2f", d);

        % Disparity sanity checks (stereo depth becomes unstable when |d| is small)
        if ~isfinite(d) || abs(d) < 1.0
            hPt3.XData = NaN; hPt3.YData = NaN; hPt3.ZData = NaN;
            msg.Value = [infoLines; "Disparity too small (< 1 px) -> depth unstable. Skipping update."];
            return;
        end
        if d < 0
            hPt3.XData = NaN; hPt3.YData = NaN; hPt3.ZData = NaN;
            msg.Value = [infoLines; "Negative disparity (uL < uR) -> wrong match or swapped L/R. Skipping update."];
            return;
        end

        Z = (f * B) / d;
        X = ((cL(1) - cx) * Z) / f;
        Y = ((cL(2) - cy) * Z) / f;

        lblX.Text = sprintf("X (m): %.3f", X);
        lblY.Text = sprintf("Y (m): %.3f", Y);

        % Z from triangulation is *depth from the camera plane* (assuming parallel stereo).
        % In Blender, cameras are at z=camZ and the ball world z is (Ztrue - Zoffset),
        % so a 'world Z' estimate is: ballZ_est = camZ - Zdepth, and Ztrue_est = ballZ_est + Zoffset.
        Zdepth = Z;
        ballZ_est  = params.camZ - Zdepth;          % estimated Blender world z of the ball
        Ztrue_est  = ballZ_est + params.Zoffset;    % estimated value matching the Ztrue slider

        lblZ.Text = sprintf("Zdepth (m): %.3f   |   Ztrue est (m): %.3f", Zdepth, Ztrue_est);

        % --- Update 3D point (DO NOT cla(ax3); otherwise skipped updates leave stale point) ---
        hPt3.XData = X;
        hPt3.YData = Y;
        hPt3.ZData = Z;

        % Auto limits so the ball is always visible
        padXY = 0.25;              % meters
        padZ  = max(0.5, 0.2*Z);   % scale with distance

        xlim(ax3, sort([X - padXY, X + padXY]));
        ylim(ax3, sort([Y - padXY, Y + padXY]));
        zlim(ax3, sort([max(0, Z - padZ), Z + padZ]));

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
        if pendingUpdate
            pendingUpdate = false;
            % run one more update with latest UI values (debounced-safe)
            scheduleUpdate();
        end
    end

%% -------------------- DEBOUNCE / UI HELPERS --------------------
    function onParamChange()
        scheduleUpdate();
    end

    function onEditZ(src)
        % Keep Z slider in sync with numeric edit, then debounce update
        try
            set(sZ,'Value',src.Value);
        catch
        end
        scheduleUpdate();
    end

    function onGraySlider(src)
        % Keep gray edit in sync with slider, then debounce update
        try
            set(eGray,'Value',src.Value);
        catch
        end
        scheduleUpdate();
    end

    function onGrayEdit(src)
        % Keep gray slider in sync with numeric edit, then debounce update
        try
            set(sGray,'Value',src.Value);
        catch
        end
        scheduleUpdate();
    end

    function onZSlider(src)
        % Keep Z edit in sync with slider, then debounce update
        try
            set(eZ,'Value',src.Value);
        catch
        end
        scheduleUpdate();
    end

    function scheduleUpdate()
        % Debounce: restart the single-shot timer so rapid changes coalesce
        try
            if ~isempty(autoTimer) && isvalid(autoTimer)
                stop(autoTimer);
                start(autoTimer);
            else
                safeUpdateOnce();
            end
        catch
            safeUpdateOnce();
        end
    end

    function onClose()
        % Stop timer, close TCP client, then close UI
        try
            if ~isempty(autoTimer) && isvalid(autoTimer)
                stop(autoTimer);
                delete(autoTimer);
            end
        catch
        end
        try
            if ~isempty(client) && isvalid(client)
                clear client; % closes tcpclient
            end
        catch
        end
        delete(ui);
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

        % Discard any stale bytes left in the TCP buffer (prevents stream desync)
        flushClient(c);

        % Local helper: retry once on timeout by dropping/recreating the socket
        function out = blenderLinkRetry(varargin)
            try
                out = blenderLink(varargin{:});
            catch ME
                isTimeout = contains(ME.message,"Timeout","IgnoreCase",true) || contains(ME.message,"timed out","IgnoreCase",true);
                isReadErr = contains(ME.message,"read","IgnoreCase",true) || contains(ME.message,"connection","IgnoreCase",true);
                if isTimeout || isReadErr
                    try, clear client; catch, end
                    if ~ensureClient()
                        rethrow(ME);
                    end
                    varargin{1} = client;
                    out = blenderLink(varargin{:});
                else
                    rethrow(ME);
                end
            end
        end

        function flushClient(c2)
            try
                n = c2.NumBytesAvailable;
                if n > 0
                    read(c2, n, "uint8");
                end
            catch
            end
        end

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

            blenderLinkRetry(c, BLENDER.width, BLENDER.height, ...
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

            IL = blenderLinkRetry(c, BLENDER.width, BLENDER.height, ...
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

            IR = blenderLinkRetry(c, BLENDER.width, BLENDER.height, ...
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
            Ig = im2double(rgb2gray(Iwork));
        
            % Detect the DARK pupil (black circle) instead of bright court lines
            % (Use adaptive or Otsu on inverted image)
            Iinv = 1 - Ig;
        
            if params.grayThresh > 0
                t = params.grayThresh;
                BW = Iinv > t;
                tUsed = t;
                tType = "manual(inv)";
            else
                tUsed = graythresh(Iinv);      % Otsu on inverted
                BW = imbinarize(Iinv, tUsed);
                tType = "otsu(inv)";
            end
        
            % Clean up and fill
            BW = bwareaopen(BW, 50);          % remove tiny specks
            BW = imclose(BW, strel('disk', 3));
            BW = imfill(BW, 'holes');
        
            % --- area limits: if yours are too strict, auto widen for this method ---
            minA = params.minBlobArea;
            maxA = params.maxBlobArea;
        
            % For the pupil, the blob is smaller than the full ball, so relax defaults
            if isempty(minA) || minA > 2000, minA = 80;  end
            if isempty(maxA) || maxA < 3000, maxA = 8000; end
        
            CC = bwconncomp(BW);
            stats = regionprops(CC,'Area','Centroid','Eccentricity');
        
            if isempty(stats)
                dbg = [
                    "GrayThresh(dark pupil): no components."
                    sprintf("t=%s %.3f", tType, tUsed)
                ];
                centroid = [NaN NaN];
                return;
            end
        
            A  = [stats.Area];
            E  = [stats.Eccentricity];
        
            ok = (A >= minA) & (A <= maxA) & (E <= 0.85);  % prefer round-ish blobs
        
            if ~any(ok)
                dbg = [
                    "GrayThresh(dark pupil): no blobs after area/ecc filter."
                    sprintf("t=%s %.3f  areas=[%.0f..%.0f]  kept=0/%d", tType, tUsed, min(A), max(A), numel(A))
                    sprintf("minA=%.0f maxA=%.0f", minA, maxA)
                ];
                centroid = [NaN NaN];
                return;
            end
        
            % If you have a refCentroid (last good), pick closest; else pick largest
            candIdx = find(ok);
            centCand = reshape([stats(candIdx).Centroid],2,[])';
            areaCand = A(candIdx);
        
            if isfield(params,'refCentroid') && ~isempty(params.refCentroid)
                d2 = sum((centCand - params.refCentroid(:).').^2, 2);
                [~,k] = min(d2);
            else
                [~,k] = max(areaCand);
            end
        
            centroid = centCand(k,:);
        
            dbg = [
                sprintf("GrayThresh: %s t=%.3f (dark pupil)", tType, tUsed)
                sprintf("Components=%d, kept=%d, chosenArea=%.0f", numel(A), numel(candIdx), areaCand(k))
            ];
            return;
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

                % Narrower radius range + metric output improves stability
                % (tune [18 70] if your rendered ball appears bigger/smaller)
                [centers, radii, metric] = imfindcircles(Ig,[18 70], ...
                    'ObjectPolarity','bright','Sensitivity',0.95,'EdgeThreshold',0.10); %#ok<ASGLU>

                if isempty(centers)
                    dbg = ["imfindcircles found nothing."];
                    return;
                end

                % Pick circle closest to previous centroid (prevents L/R flip and jitter)
                if isfield(params,'refCentroid') && ~isempty(params.refCentroid) && size(centers,1) > 1
                    d2 = sum((centers - params.refCentroid(:).').^2, 2);
                    [~,k] = min(d2);
                else
                    [~,k] = max(metric);
                end

                centroid = centers(k,:);
                dbg(end+1) = sprintf("Circles: picked r=%.1f metric=%.3f", radii(k), metric(k));
                return;

            otherwise
                dbg = ["Unknown method."];
                return;
        end

        % quick sanity info
        dbg(end+1) = sprintf("BW white pixels = %d", nnz(BW));

        % blob filter + shape scoring (more robust when court lines also threshold)
        % 1) Remove tiny specks
        BW = bwareaopen(BW, max(1, round(params.minBlobArea)));

        % 2) Morphology to suppress thin lines and consolidate the ball region
        try
            BW = imopen(BW, strel('disk', 2));     % removes thin structures (court lines)
            BW = imclose(BW, strel('disk', 3));    % closes small gaps on the ball edge
            BW = imfill(BW, 'holes');              % fill donut-like ball regions
        catch
        end

        % 3) Score connected components by "ball-likeness"
        stats = regionprops(BW,'Area','Centroid','Perimeter','Eccentricity','Solidity');

        if isempty(stats)
            dbg(end+1) = "No blobs after threshold/area filter.";
            return;
        end

        areas = [stats.Area];

        % User area constraints (if provided)
        minA = max(1, params.minBlobArea);
        maxA = params.maxBlobArea;

        % If maxBlobArea is not sensible, set a generous default relative to image size
        if isempty(maxA) || ~isfinite(maxA) || maxA <= 0
            maxA = 0.25 * numel(BW);  % up to 25% of image
        end

        okA = (areas >= minA) & (areas <= maxA);
        statsA = stats(okA);

        if isempty(statsA)
            dbg(end+1) = "Blobs found, but none within [min,max] area.";
            % fall back: keep all and let shape score decide
            statsA = stats;
        end

        % Compute circularity (1 = perfect circle). Protect from divide-by-zero.
        per = [statsA.Perimeter];
        circ = 4*pi*[statsA.Area] ./ max(per.^2, eps);

        ecc  = [statsA.Eccentricity];
        sol  = [statsA.Solidity];

        % Shape filter: ball should be fairly round + solid
        okShape = (circ >= 0.35) & (ecc <= 0.92) & (sol >= 0.75);

        if any(okShape)
            statsB = statsA(okShape);
            per = [statsB.Perimeter];
            circ = 4*pi*[statsB.Area] ./ max(per.^2, eps);
        else
            statsB = statsA; % no shape match; pick best by score anyway
        end

        % Score: prioritize circularity, then area (avoid picking long lines)
        score = circ .* sqrt([statsB.Area]);

        % Prefer the blob closest to the previous centroid (stabilizes tracking), if available
        if isfield(params,'refCentroid') && ~isempty(params.refCentroid) && numel(statsB) > 1
            C = reshape([statsB.Centroid], 2, []).';   % Nx2
            d2 = sum((C - params.refCentroid(:).').^2, 2);
            % combine shape score with proximity (normalize both)
            s1 = score(:) / max(score(:));
            s2 = 1 ./ (1 + d2);                         % higher is closer
            comb = 0.75*s1 + 0.25*s2;
            [~,k] = max(comb);
        else
            [~,k] = max(score);
        end
        c = statsB(k).Centroid;

        dbg(end+1) = sprintf("Blob pick: area=%d  circ=%.2f  ecc=%.2f  sol=%.2f", ...
                             round(statsB(k).Area), circ(k), statsB(k).Eccentricity, statsB(k).Solidity);

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
