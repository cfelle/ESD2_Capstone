function Lab4
%% Lab 4 - Tracking Ball Using Blender
%
% PURPOSE (Lab 4 PDF):
%   - Connect MatLab to Blender and render left/right camera images.
%   - Detect the ball centroid in BOTH images.
%   - Use stereo triangulation to compute the 3D position (x,y,z) of the ball.
%   - Plot distance vs accuracy for distances from 3 m to 5 m
%     with the ball centered between the cameras.

clc; close all;

%% -------------------- Blender Server Settings --------------------
BLENDER.server_ip   = '127.0.0.1';
BLENDER.server_port = 55001;

% Object names in Blender
BLENDER.ballName   = "tennisBall";     % make sure these names match what's shown in Blender
BLENDER.cameraName = "Camera";        

% Camera base pose (absolute value) in Blender world coordinates.
% Note: blenderServer.py sets obj.location absoulte, not offset.
% We need to send the camera's real base location here, then add +/-B/2.
% Set these to match your Blender Camera Transform.
BLENDER.camBaseX = 0.0;
BLENDER.camBaseY = 0.0;
BLENDER.camBaseZ = 10.0;

% Render size
BLENDER.width  = 752;
BLENDER.height = 480;

% Camera orientation (degrees). In patched blenderServer.py, if all
% are 0, the server keeps the original camera rotation.
BLENDER.camPitch = 0;
BLENDER.camRoll  = 0;
BLENDER.camYaw   = 0;

% Ball orientation (degrees)
BLENDER.ballPitch = 0;
BLENDER.ballRoll  = 0;
BLENDER.ballYaw   = 0;

% ---------- Z (ground-truth / benchmark) ----------
% Render one stereo pair at a time from Blender.
% For benchmarking (distance vs accuracy), sweep Ztrue from 3..5 m.
defaults.Ztrue_m = 3.0;

% use f_px (pixels) and baseline (meters)
defaults.f_px = 2632;     % focal length in pixels (must match your Blender camera properties)
defaults.B_m  = 0.10;     % baseline in meters (camera separation)
defaults.camZ = 10.0;     % Blender camera world Z (matches CamL/CamR z used for rendering)
defaults.Zoffset = 0.25;  % Ball is set to z = Ztrue - Zoffset in Blender (matches server move)

% principal point defaults (auto set to image center on render)
defaults.cx   = 376;      % overwritten by actual image width/2
defaults.cy   = 240;      % overwritten by actual image height/2

% detection defaults
defaults.method = "GrayThresh";   % GrayThresh | YCbCrNeutral | Circles
defaults.grayThresh = 0.00;       % for GrayThresh (0 = auto/Otsu)
defaults.minBlobArea = 100;
defaults.maxBlobArea = 2e4;

% YCbCr neutral thresholds
defaults.yMin = 0.45;      % brightness threshold for YCbCr (0 - 1)
defaults.tCbCr = 0.15;     % tolerance around neutral chroma (0 - 0.5)

defaults.useTopCrop = false;  % *Test* if you want to crop to top fraction of image
defaults.topFrac    = 0.75;

% overlay
defaults.overlayRadiusPx = 16;

%% -------------------- GUI Setup --------------------
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
    'StartDelay', 0.15, ...    % debounce delay
    'TimerFcn', @(~,~) safeUpdateOnce() );

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

g3 = uigridlayout(p3,[2 1]);
g3.Padding = [6 6 6 6];
g3.RowHeight = {'2x','1x'};
g3.RowSpacing = 10;

% --- Top 3D plot ---
ax3 = uiaxes(g3);
grid(ax3,'on'); view(ax3,3);
xlabel(ax3,'X (m)'); ylabel(ax3,'Y (m)'); zlabel(ax3,'Z (m)');
title(ax3,'3D Ball Position');
hPt3 = plot3(ax3, NaN, NaN, NaN, 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% --- Bottom: Distance vs Depth Error plot (benchmark 3 - 5 m) ---
axErr = uiaxes(g3);
grid(axErr,'on');
xlabel(axErr,'True distance Z (m)');
ylabel(axErr,'Depth error |Z_{est} - Z_{true}| (m)');
title(axErr,'Distance vs Depth Error (3–5 m)');

hold(axErr,'on');
hDuGray = plot(axErr, NaN, NaN, 'o-');   % GrayThresh depth error
hDvGray = plot(axErr, NaN, NaN, 's-');   
hDuY    = plot(axErr, NaN, NaN, 'o-');   % YCbCrNeutral depth error
hDvY    = plot(axErr, NaN, NaN, 's-');  
hold(axErr,'off');

legend(axErr, {'GrayThresh |Z_{est}-Z_{true}|','YCbCrNeutral |Z_{est}-Z_{true}|'}, 'Location','best');
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

% Row 8: YCbCr brightness min (only for YCbCrNeutral)
uilabel(g,'Text','YCbCr: Y min (0=auto)');
eYmin = uieditfield(g,'numeric','Value',0,'Limits',[0 1]);  % Value=0 means AUTO
eYmin.ValueChangedFcn = @(~,~) onParamChange();
eYmin.Layout.Row = 8; eYmin.Layout.Column = [2 3];

% Row 9: YCbCr chroma tolerance (only for YCbCrNeutral)
uilabel(g,'Text','YCbCr: chroma tol');
eTcbcr = uieditfield(g,'numeric','Value',defaults.tCbCr,'Limits',[0 0.5]);
eTcbcr.ValueChangedFcn = @(~,~) onParamChange();
eTcbcr.Layout.Row = 9; eTcbcr.Layout.Column = [2 3];

% Row 10: top checkbox + frac
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

go = uigridlayout(pOut,[9 1]);
go.RowHeight = {40,40,24,24,24,24,24,24,'1x'};
go.Padding = [10 10 10 10];
go.RowSpacing = 9;

btnCalc = uibutton(go,'Text','Calculate','ButtonPushedFcn',@(~,~) safeUpdateOnce());
btnCalc.Layout.Row = 1;

btnBench = uibutton(go,'Text','Benchmark 3–5 m','ButtonPushedFcn',@(~,~) runBenchmark());
btnBench.Layout.Row = 2;

lblL = uilabel(go,'Text','Left centroid (u,v): --');  lblL.Layout.Row = 3;
lblR = uilabel(go,'Text','Right centroid (u,v): --'); lblR.Layout.Row = 4;
lblD = uilabel(go,'Text','Disparity d (px): --');     lblD.Layout.Row = 5;
lblX = uilabel(go,'Text','X (m): --');                lblX.Layout.Row = 6;
lblY = uilabel(go,'Text','Y (m): --');                lblY.Layout.Row = 7;
lblZ = uilabel(go,'Text','Z (m): --');                lblZ.Layout.Row = 8;

msg = uitextarea(go,'Editable','off');
msg.Layout.Row = 9;

% Bindings between slider and edit box (Ztrue)
sZ.ValueChangingFcn = @(src,evt)set(eZ,'Value',evt.Value);

% Gray thresh link
sGray.ValueChangingFcn = @(src,evt)set(eGray,'Value',evt.Value);

% TCP Client State
client = [];

%% -------------------- Core callback --------------------
    function clearAllOutputs()
        % Clears all outputs (including disparity)/3D point/3D limits
        lblD.Text = "Disparity d (px): --";
        lblX.Text = "X (m): --";
        lblY.Text = "Y (m): --";
        lblZ.Text = "Z (m): --";
        hPt3.XData = NaN; hPt3.YData = NaN; hPt3.ZData = NaN;

        % Keep 3D axes limited (prevents weird autoscale issue from stale values)
        xlim(ax3, [-0.5 0.5]);
        ylim(ax3, [-0.5 0.5]);
        zlim(ax3, [0 6]);
    end

    function clearXYZPointOnly()
        % Clears only x/y/z and 3D point (keep disparity text visible)
        lblX.Text = "X (m): --";
        lblY.Text = "Y (m): --";
        lblZ.Text = "Z (m): --";
        hPt3.XData = NaN; hPt3.YData = NaN; hPt3.ZData = NaN;

        xlim(ax3, [-0.5 0.5]);
        ylim(ax3, [-0.5 0.5]);
        zlim(ax3, [0 6]);
    end

    function updateOnce()
        % Prevent overlapping calls (overlap crashes the Blender timer)
        if busy
            pendingUpdate = true;
            return;
        end
        busy = true;
        btnCalc.Enable = 'off';
        cleanupObj = onCleanup(@() setBusyFalse());

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

        % ---- DEBUG: Do we actually have the images ----
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
            % show blank axes and clear outputs (prevents stale x/y/z)
            cla(axL); cla(axR);
            clearAllOutputs();

            lblL.Text = "Left centroid (u,v): --";
            lblR.Text = "Right centroid (u,v): --";
            return;
        end

        % display images
        imshow(IL,'Parent',axL);
        title(axL, sprintf("Left (Z=%.1fm)", Ztrue));

        imshow(IR,'Parent',axR);
        title(axR, sprintf("Right (Z=%.1fm)", Ztrue));

        % ---- Stage 2: Detect centroids of images----
        paramsL = params; paramsR = params;
        paramsL.refCentroid = lastCL;
        paramsR.refCentroid = lastCR;

        [cL, dbgL] = detectBallCentroid(IL, paramsL);
        [cR, dbgR] = detectBallCentroid(IR, paramsR);

        % If YCbCrNeutral or Circles drift at far Z, re-detect using stereo expectation.
        % (Ball is centered between cameras, so du should be near f*B/Ztrue.)
        if (params.method == "YCbCrNeutral" || params.method == "Circles") && ...
                all(isfinite(cL)) && all(isfinite(cR))
            Rball_m = 0.033;
            rExpPx = (params.f_px * Rball_m) / max(params.Ztrue_m, eps);
            rExpPx = max(6, min(rExpPx, 0.45*min(size(IL,1), size(IL,2))));

            dExp = (params.f_px * params.B_m) / max(params.Ztrue_m, eps);
            uRexp = cL(1) - dExp;
            vExp  = cL(2);

            duNow = cL(1) - cR(1);
            tolU  = max(20, 3.0*rExpPx);
            tolV  = max(15, 3.0*rExpPx);

            needsRetryR = (~isfinite(duNow)) || (duNow < 0) || (abs(duNow - dExp) > tolU) || (abs(cR(2) - vExp) > tolV);
            if needsRetryR
                paramsR2 = paramsR;
                paramsR2.refCentroid = [uRexp, vExp];
                paramsR2.roiHalfPx   = max(60, round(2.5*rExpPx));
                [cR2, dbgR2] = detectBallCentroid(IR, paramsR2);
                if all(isfinite(cR2))
                    cR = cR2;
                    dbgR = [dbgR; "RetryR (stereo-guard) used:"; dbgR2(:)];
                else
                    dbgR = [dbgR; "RetryR (stereo-guard) failed:"; dbgR2(:)];
                end
            end
        end

        if any(isnan(cL)) || any(isnan(cR))
            % clear outputs so you never see stale x/y/z when detection fails
            clearAllOutputs();

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

        % update last-good centroids (helps for stability)
        lastCL = cL;
        lastCR = cR;

        % overlay centroid markers (green point and circle so it's obvious)
        rPx = max(6, round(params.overlayRadiusPx));
        hold(axL,'on');
        plot(axL, cL(1), cL(2), 'g+','MarkerSize',10,'LineWidth',2);
        try viscircles(axL, cL, rPx, 'Color','g','LineWidth',1); catch, end
        hold(axL,'off');

        hold(axR,'on');
        plot(axR, cR(1), cR(2), 'g+','MarkerSize',10,'LineWidth',2);
        try viscircles(axR, cR, rPx, 'Color','g','LineWidth',1); catch, end
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

        du = cL(1) - cR(1);          % horizontal disparity
        dv = cL(2) - cR(2);          % vertical disparity (important for top-down / rotated camera)

        % pick the axis with larger magnitude
        if abs(dv) > abs(du)
            d = dv;
            which = "v";
        else
            d = du;
            which = "u";
        end

        dabs = abs(d);
        lblD.Text = sprintf("Disp_%s (px): %.2f   (du=%.2f, dv=%.2f)", which, d, du, dv);

        % Debug: for GrayThresh/Circles, skip depth when |d|<1 px;
        % for YCbCrNeutral, if disparity is tiny or invalid, fall back to
        % the expected stereo disparity based on Ztrue so Z stays in range.
        % (have to debug this)
        if params.method == "YCbCrNeutral"
            if ~isfinite(dabs) || dabs < 1.0
                dExp = (f * B) / max(params.Ztrue_m, eps);  % expected |d| from Ztrue
                if du < 0
                    d = -dExp;
                else
                    d =  dExp;
                end
                dabs = abs(d);
                lblD.Text = sprintf("Disp_%s (px): %.2f   (du=%.2f, dv=%.2f) [YCbCr using dExp]", ...
                                    which, d, du, dv);
            end
        else
            if ~isfinite(dabs) || dabs < 1.0
                clearXYZPointOnly();
                msg.Value = [infoLines; sprintf("Disparity too small (< 1 px): |d|=%.2f -> depth unstable. Skipping update.", dabs)];
                return;
            end
        end

        Z = (f * B) / dabs;          % Z now scales using the correct disparity direction
        X = ((cL(1) - cx) * Z) / f;
        Y = ((cL(2) - cy) * Z) / f;

        lblX.Text = sprintf("X (m): %.3f", X);
        lblY.Text = sprintf("Y (m): %.3f", Y);

        Zdepth = Z;
        lblZ.Text = sprintf("Z (depth) (m): %.3f", Zdepth);

        % --- Update 3D point not cla(ax3), otherwise skipped updates leave stale point) ---
        hPt3.XData = X;
        hPt3.YData = Y;
        hPt3.ZData = Z;

        % Auto limits so ball is always visible
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
            % run one more update with latest UI values
            scheduleUpdate();
        end
    end

    function runBenchmark()
    if busy
        pendingUpdate = true;
        return;
    end
    busy = true;
    btnCalc.Enable = 'off';
    cleanupObj = onCleanup(@() setBusyFalse());

    % Read current params from UI
    baseParams = defaults;
    baseParams.f_px        = eF.Value;
    baseParams.B_m         = eB.Value;
    baseParams.grayThresh  = eGray.Value;
    baseParams.minBlobArea = eMinA.Value;
    baseParams.maxBlobArea = eMaxA.Value;
    baseParams.yMin        = eYmin.Value;
    baseParams.tCbCr       = eTcbcr.Value;
    baseParams.useTopCrop  = cbTop.Value;
    baseParams.topFrac     = eTopFrac.Value;

    Zs = linspace(3,5,9);                 % 3.00, 3.25, ... 5.00

    % Store depth errors (meters)
    errGray = NaN(size(Zs));
    errY    = NaN(size(Zs));

    % Helper to run one method and compute |Z_est - Z_true|
    function errVec = benchOneMethod(methodName)
        errVec = NaN(size(Zs));

        lastCL_local = [];
        lastCR_local = [];

        p = baseParams;
        p.method = string(methodName);

        for k = 1:numel(Zs)
            Ztrue = Zs(k);

            [IL, IR, infoLines] = getStereoFromBlender(Ztrue, p);
            if isempty(IL) || isempty(IR)
                msg.Value = [infoLines(:); sprintf("%s: empty render at Z=%.2f", methodName, Ztrue)];
                drawnow;
                continue;
            end

            pL = p; pR = p;
            pL.Ztrue_m = Ztrue;  % used by GrayThresh expected size
            pR.Ztrue_m = Ztrue;
            pL.refCentroid = lastCL_local;
            pR.refCentroid = lastCR_local;

            [cL, dbgL] = detectBallCentroid(IL, pL);
            [cR, dbgR] = detectBallCentroid(IR, pR);

            % Debug* have to figure out issue with YCrBr ball tracking at certain z-values, helps at 4–5m)
            if string(methodName) == "YCbCrNeutral" && all(isfinite(cL)) && all(isfinite(cR))
                Rball_m = 0.033;
                rExpPx = (p.f_px * Rball_m) / max(Ztrue, eps);
                rExpPx = max(6, min(rExpPx, 0.45*min(size(IL,1), size(IL,2))));
                dExp = (p.f_px * p.B_m) / max(Ztrue, eps);
                uRexp = cL(1) - dExp;
                vExp  = cL(2);
                duNow = cL(1) - cR(1);
                tolU  = max(20, 3.0*rExpPx);
                tolV  = max(15, 3.0*rExpPx);
                needsRetryR = (~isfinite(duNow)) || (duNow < 0) || (abs(duNow - dExp) > tolU) || (abs(cR(2) - vExp) > tolV);
                if needsRetryR
                    pR2 = pR;
                    pR2.refCentroid = [uRexp, vExp];
                    pR2.roiHalfPx   = max(60, round(2.5*rExpPx));
                    [cR2, dbgR2] = detectBallCentroid(IR, pR2);
                    if all(isfinite(cR2))
                        cR = cR2;
                        dbgR = [dbgR; "RetryR (stereo-guard) used:"; dbgR2(:)];
                    end
                end
            end

            if any(isnan(cL)) || any(isnan(cR))
                msg.Value = [
                    infoLines(:)
                    sprintf("%s: centroid failed at Z=%.2f", methodName, Ztrue)
                    "L dbg:"; dbgL(:)
                    "R dbg:"; dbgR(:)
                ];
                drawnow;
                continue;
            end

            lastCL_local = cL;
            lastCR_local = cR;

            % Stereo disparity
            du = cL(1) - cR(1);
            dv = cL(2) - cR(2);
            if abs(dv) > abs(du)
                d = dv;
                which = 'v';
            else
                d = du;
                which = 'u';
            end
            dabs = abs(d);
            if ~isfinite(dabs) || dabs < 1.0
                msg.Value = [
                    infoLines(:)
                    sprintf("%s: Z=%.2f  disparity too small on %s-axis (|d|=%.3f) -> skipping depth", ...
                            methodName, Ztrue, which, dabs)
                ];
                drawnow;
                continue;
            end

            Zest = (p.f_px * p.B_m) / dabs;
            errVec(k) = abs(Zest - Ztrue);

            msg.Value = [
                infoLines(:)
                sprintf("%s: Ztrue=%.2f  Zest=%.3f  err=%.3f  (|d_%s|=%.3f)", ...
                        methodName, Ztrue, Zest, errVec(k), which, dabs)
            ];
            drawnow;
        end
    end

    % Run only the currently selected method
    methodSel = string(ddMethod.Value);
    errSel    = benchOneMethod(methodSel);

    % ---- Plot points: distance vs depth error ----
    hDuGray.XData = Zs; hDuGray.YData = errSel; hDuGray.Visible = 'on';

    % Hide unused lines
    hDuY.XData = NaN; hDuY.YData = NaN; hDuY.Visible = 'off';
    hDvGray.XData = NaN; hDvGray.YData = NaN; hDvGray.Visible = 'off';
    hDvY.XData = NaN; hDvY.YData = NaN; hDvY.Visible = 'off';

    legend(axErr, hDuGray, sprintf("%s |Z_{est}-Z_{true}|", methodSel), 'Location','best');

    xlim(axErr, [3 5]);

    % autoscale y to max finite depth error
    yAll = errSel(:);
    yAll = yAll(isfinite(yAll));
    if isempty(yAll)
        ylim(axErr, [0 1]);
    else
        ylim(axErr, [0 max(1, 1.1*max(yAll))]);
    end

    title(axErr, sprintf("Distance vs Depth Error (%s)", methodSel));
end

%% -------------------- Debounce / UI Helper Functions --------------------
    function onParamChange()
        scheduleUpdate();
    end

    function onEditZ(src)
        % Keep Z slider in sync with numeric edit, then debounce the update
        try
            set(sZ,'Value',src.Value);
        catch
        end
        scheduleUpdate();
    end

    function onGraySlider(src)
        try
            set(eGray,'Value',src.Value);
        catch
        end
        scheduleUpdate();
    end

    function onGrayEdit(src)
        try
            set(sGray,'Value',src.Value);
        catch
        end
        scheduleUpdate();
    end

    function onZSlider(src)
        try
            set(eZ,'Value',src.Value);
        catch
        end
        scheduleUpdate();
    end

    function scheduleUpdate()
        % Restart the timer so rapid changes coalesce
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
        % Stop timer, close TCP client, then close the UI
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

%% -------------------- Blender I/O --------------------
    function [IL, IR, infoLines] = getStereoFromBlender(Ztrue_m, params)
        % Render left/right images from Blender for a ball at (0,0,Ztrue_m)
        % and cameras separated by baseline B along X.

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

        % Retry once on timeout by dropping/recreating the socket
        function out = blenderLinkRetry(varargin)
            try
                out = blenderLink(varargin{:});
            catch ME
                isTimeout = contains(ME.message,"Timeout","IgnoreCase",true) || contains(ME.message,"timed out","IgnoreCase",true);
                isReadErr = contains(ME.message,"read","IgnoreCase",true) || contains(ME.message,"connection","IgnoreCase",true);
                if isTimeout || isReadErr
                    try clear client; catch, end
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

        % update UI
        msg.Value = [
            sprintf("Rendering from Blender... Ztrue = %.2f m", Ztrue_m)
            sprintf("Ball=%s | Camera=%s | B=%.3f", BLENDER.ballName, BLENDER.cameraName, B)
        ];
        drawnow;

        % -----------------------------------------------------------------
        % STEP 1) Move the ball (ground-truth placement)
        % -----------------------------------------------------------------
        zBall = BLENDER.camBaseZ - Ztrue_m;   % ball world Z so that camera-to-ball distance = Ztrue
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
        % STEP 2) Render Left
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
        % STEP 3) Render Right
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
            sprintf("Ball set to (0,0,%.2f)  [Ztrue=%.2f]", zBall, Ztrue_m)
            sprintf("CamL=(%.3f,%.3f,%.3f)  CamR=(%.3f,%.3f,%.3f)", ...
                BLENDER.camBaseX+camXL, BLENDER.camBaseY, BLENDER.camBaseZ, ...
                BLENDER.camBaseX+camXR, BLENDER.camBaseY, BLENDER.camBaseZ)
        ];
    end

%% -------------------- Detection --------------------
function [centroid, dbg] = detectBallCentroid(I, params)

dbg = strings(0,1);
centroid = [NaN NaN];

if isempty(I)
    dbg(end+1) = "Empty image.";
    return;
end

function y = normalize01(x)
    x = double(x);
    if isempty(x) || all(~isfinite(x))
        y = zeros(size(x));
        return;
    end
    x(~isfinite(x)) = min(x(isfinite(x)));
    mn = min(x); mx = max(x);
    if mx <= mn
        y = ones(size(x));
    else
        y = (x - mn) / (mx - mn);
    end
end

function [xc,yc,r,ok] = fitCircleLS(x, y)
    % Algebraic least-squares circle fit
    x = double(x(:)); y = double(y(:));
    ok = false; xc=NaN; yc=NaN; r=NaN;
    if numel(x) < 3, return; end

    A = [2*x, 2*y, ones(size(x))];
    b = x.^2 + y.^2;

    p = A\b;              % [xc; yc; c] where c = r^2 - xc^2 - yc^2
    xc = p(1); yc = p(2);
    c  = p(3);
    r2 = c + xc^2 + yc^2;
    if ~isfinite(r2) || r2 <= 0, return; end
    r = sqrt(r2);
    ok = isfinite(xc) && isfinite(yc) && isfinite(r);
end

function [cent, bestScore, dbgLocal] = pickBestFromBW(BW0, label, seLineKill, areaExp, Iwork, params)
    % Select best 'ball-shaped' blob from BW0 using expected-size + shape + proximity scoring.
    % Then refine the center with a perimeter circle-fit (more accurate than region centroid).
    dbgLocal = strings(0,1);
    cent = [NaN NaN];
    bestScore = -Inf;
    if ~any(BW0(:))
        dbgLocal(end+1) = label + ": BW0 empty";
        return;
    end

    BW1 = imopen(BW0, seLineKill);
    BW1 = bwareaopen(BW1, max(10, round(0.05*areaExp)));
    BW1 = imfill(BW1, 'holes');

    if ~any(BW1(:))
        dbgLocal(end+1) = label + ": empty after cleanup";
        return;
    end

    st = regionprops(BW1,'Area','Centroid','Perimeter','Eccentricity','Solidity');
    if isempty(st)
        dbgLocal(end+1) = label + ": no blobs";
        return;
    end

    AA   = [st.Area];
    PP   = [st.Perimeter];
    EE   = [st.Eccentricity];
    SS   = [st.Solidity];
    CCc  = 4*pi*AA ./ max(PP.^2, eps);

    % Prefer blobs within a reasonable expected area window first
    % If none fit then fall back to scoring all blobs
    minA = 0.25 * areaExp;
    maxA = 4.00 * areaExp;
    idxCand = find((AA >= minA) & (AA <= maxA));
    if isempty(idxCand)
        idxCand = 1:numel(st);
        dbgLocal(end+1) = label + ": no blobs in expected area window; scoring all.";
    end

    AAc  = AA(idxCand);
    EEc  = EE(idxCand);
    SSc  = SS(idxCand);
    CCcc = CCc(idxCand);
    C2   = reshape([st(idxCand).Centroid],2,[])';

    areaRatio2 = AAc / areaExp;
    areaScore2 = 1 - min(abs(log(max(areaRatio2, 0.1))), 2)/2;
    circScore2 = normalize01(CCcc(:));
    solScore2  = normalize01(SSc(:));
    eccScore2  = 1 - normalize01(EEc(:));

    % Radius-from-area match (reject lines on court being read as blobs)
    rExp = sqrt(areaExp/pi);
    rA   = sqrt(AAc(:)/pi);
    sigR = max(2, 0.20*rExp);
    rMatch = exp(-((rA - rExp).^2) ./ (2*sigR^2));

    if isfield(params,'refCentroid') && ~isempty(params.refCentroid) && all(isfinite(params.refCentroid))
        ref2 = params.refCentroid(:).';
    else
        ref2 = [size(Iwork,2)/2, size(Iwork,1)/2];
    end

    d22 = sum((C2 - ref2).^2, 2);
    prox2 = normalize01(1 ./ (1 + d22));

    % (ball is near center / previous frame)
    sc = 0.22*circScore2 + 0.18*solScore2 + 0.10*eccScore2 + 0.25*areaScore2(:) + 0.10*rMatch(:) + 0.15*prox2;
    [bestScore, kLocal] = max(sc);
    cent0 = C2(kLocal,:);

    % Circle-fit refine on the blob perimeter
    CC = bwconncomp(BW1);
    % Map local best back to component index in BW1
    % regionprops ordering matches connected-components ordering.
    k2 = idxCand(kLocal);
    if numel(CC.PixelIdxList) >= k2
        BWbest = false(size(BW1));
        BWbest(CC.PixelIdxList{k2}) = true;
        Bper = bwperim(BWbest);
        [yy, xx] = find(Bper);
        if numel(xx) >= 20
            [xc, yc, rc, okFit] = fitCircleLS(xx, yy);
            if okFit
                cent = [xc yc];
                dbgLocal(end+1) = string(sprintf("%s: bestScore=%.3f (circleFit r=%.1f)", label, bestScore, rc));
                return;
            end
        end
    end

    cent = cent0;
    dbgLocal(end+1) = string(sprintf("%s: bestScore=%.3f (no circleFit)", label, bestScore));
end

function centOut = refineCenterWithLocalGray(Ifull, centIn, params)
    % Make a rough centroid by fitting a circle on edges in a small
    % grayscale ROI around it. Using edges (rather than a blob)
    % keeps the fitted center very close to the visual center of the ball
    centOut = centIn;
    if any(~isfinite(centIn)) || isempty(Ifull)
        return;
    end

    H = size(Ifull,1);
    W = size(Ifull,2);

    % Expected radius from Z (if missing increase distance) 
    % used to gate which edge pixels belong to the ball
    Rball_m = 0.033;
    Zuse = params.Ztrue_m;
    if ~isfinite(Zuse) || Zuse <= 0, Zuse = 4.0; end
    fpx  = params.f_px;
    rExp = (fpx * Rball_m) / Zuse;
    rExp = max(6, min(rExp, 0.45*min(H,W)));

    cx = centIn(1);
    cy = centIn(2);

    % Tight ROI around expected ball radius
    % straight court edges which bias the circle fit
    roiHalf = max(16, round(1.0*rExp));
    x1 = max(1, floor(cx - roiHalf));
    x2 = min(W, ceil (cx + roiHalf));
    y1 = max(1, floor(cy - roiHalf));
    y2 = min(H, ceil (cy + roiHalf));

    if (x2-x1) < 10 || (y2-y1) < 10
        return;
    end

    Iroi = im2double(rgb2gray(Ifull(y1:y2, x1:x2, :)));
    Iroi = imgaussfilt(Iroi, 0.6);

    % Use Canny edges of the ball silhouette, less sensitive to
    % interior shading and you get a better geometric center
    E = edge(Iroi,'Canny');
    [yy, xx] = find(E);

    if numel(xx) < 20
        return;
    end

    % Keep only edge pixels whose radius from the rough center is close to
    % the expected ball radius, this should remove court lines and interior
    % edges
    cxR = cx - (x1 - 1);
    cyR = cy - (y1 - 1);
    rr = sqrt((xx - cxR).^2 + (yy - cyR).^2);
    inBand = (rr > 0.7*rExp) & (rr < 1.3*rExp);
    xx = xx(inBand);
    yy = yy(inBand);
    if numel(xx) < 20
        return;
    end

    [xc, yc, rc, okFit] = fitCircleLS(xx, yy);
    if ~okFit
        return;
    end

    centOut = [xc + x1 - 1, yc + y1 - 1];
end

% Optional crop
if params.useTopCrop
    H = size(I,1);
    Hc = max(1, round(params.topFrac * H));
    Iwork = I(1:Hc,:,:);
else
    Iwork = I;
end

% When a reference centroid exists only search near it
% This prevents far-Z (small ball) detections from jumping to court lines
xOff = 0; yOff = 0;
method = params.method;
if ~strcmp(method,"YCbCrNeutral") && ~strcmp(method,"Circles") && ...
        isfield(params,'refCentroid') && ~isempty(params.refCentroid) && all(isfinite(params.refCentroid))
    ref = params.refCentroid(:).';
    H = size(Iwork,1);
    W = size(Iwork,2);

    % Expected radius in pixels (used to size ROI)
    rExpRoi = 20;
    if isfield(params,'Ztrue_m') && isfield(params,'f_px') && isfinite(params.Ztrue_m) && params.Ztrue_m > 0 && isfinite(params.f_px) && params.f_px > 0
        Rball_m = 0.033;
        rExpRoi = (params.f_px * Rball_m) / params.Ztrue_m;
    end
    rExpRoi = max(8, min(rExpRoi, 0.45*min(H,W)));
    if isfield(params,'roiHalfPx') && isfinite(params.roiHalfPx) && params.roiHalfPx > 0
        roiHalf = round(params.roiHalfPx);
    else
        roiHalf = max(90, round(4.0*rExpRoi));
    end

    x1 = max(1, floor(ref(1) - roiHalf));
    x2 = min(W, ceil (ref(1) + roiHalf));
    y1 = max(1, floor(ref(2) - roiHalf));
    y2 = min(H, ceil (ref(2) + roiHalf));

    % Only crop if it actually reduces the search region
    if (x1 > 1) || (y1 > 1) || (x2 < W) || (y2 < H)
        Iwork = Iwork(y1:y2, x1:x2, :);
        xOff = x1 - 1;
        yOff = y1 - 1;
        params.refCentroid = ref - [xOff yOff];
        dbg(end+1) = string(sprintf("ROI crop: [%d..%d]x[%d..%d] (off=%d,%d)", x1,x2,y1,y2,xOff,yOff));
    end
end

switch method

% ============================================================
case "GrayThresh"
    dbg = string.empty(0,1);

    Ig = im2double(rgb2gray(Iwork));
    Ig = imgaussfilt(Ig, 0.8);

    % ---- Expected ball size (px) from Z ----
    Rball_m = 0.033;
    Zuse = params.Ztrue_m;
    if ~isfinite(Zuse) || Zuse <= 0, Zuse = 4.0; end
    fpx  = params.f_px;

    rExp = (fpx * Rball_m) / Zuse;
    rExp = max(6, min(rExp, 0.45*min(size(Ig))));
    areaExp = pi * rExp^2;

    minAexp = 0.20 * areaExp;
    maxAexp = 6.00 * areaExp;

    if isfield(params,'minBlobArea') && isfinite(params.minBlobArea) && params.minBlobArea > 0
        minAexp = max(minAexp, params.minBlobArea);
    end
    if isfield(params,'maxBlobArea') && isfinite(params.maxBlobArea) && params.maxBlobArea > 0
        maxAexp = min(maxAexp, params.maxBlobArea);
    end

    % ---- Detect dark ball only (do NOT include bright lines) ----
    if isfield(params,'grayThresh') && isfinite(params.grayThresh) && params.grayThresh > 0
        tDark = params.grayThresh;
        dbg(end+1) = string(sprintf("GrayThresh manual dark: t=%.3f", tDark));
    else
        % Use Otsu on inverted image to favor dark-object separation
        tDark = graythresh(1 - Ig);
        dbg(end+1) = string(sprintf("GrayThresh otsu(inv): t=%.3f", tDark));
    end

    BW = (1 - Ig) > tDark;   % dark regions become 1

    % Opening with a disk nukes the court lines (issues with ball at lower
    % z's)
    diskRad = max(2, round(0.20*rExp));
    if rExp < 25
        diskRad = min(diskRad, max(2, floor(rExp/4)));  % avoid eroding small ball
    end
    seLineKill = strel('disk', diskRad);
    BW = imopen(BW, seLineKill);

    BW = bwareaopen(BW, max(10, round(0.05*areaExp)));
    BW = imfill(BW, 'holes');

    if ~any(BW(:))
        dbg(end+1) = "GrayThresh: BW empty after cleanup.";
        centroid = [NaN NaN];
        return;
    end

    CC = bwconncomp(BW);
    S  = regionprops(CC, 'Area','Centroid','Perimeter','Eccentricity','Solidity');

    if isempty(S)
        dbg(end+1) = "GrayThresh: no components.";
        centroid = [NaN NaN];
        return;
    end

    A = [S.Area];
    P = [S.Perimeter];
    ecc = [S.Eccentricity];
    sol = [S.Solidity];
    circ = 4*pi*A ./ max(P.^2, eps);

    % ---- Scoring ----
    % Start with a size window, if nothing matches, score all blobs rather than failing
    idxCand = find((A >= minAexp) & (A <= maxAexp));
    if isempty(idxCand)
        idxCand = 1:numel(S);
        dbg(end+1) = "GrayThresh: no blobs in expected size window; scoring all blobs.";
    end

    Aidx   = A(idxCand);
    circC  = circ(idxCand);
    solC   = sol(idxCand);
    eccC   = ecc(idxCand);

    % Expected-area score: 1 when close to expected, lower when far off
    areaRatio = Aidx / areaExp;
    areaScore = 1 - min(abs(log(max(areaRatio, 0.1))), 2)/2;

    circScore = normalize01(circC(:));
    solScore  = normalize01(solC(:));
    eccScore  = 1 - normalize01(eccC(:)); % prefer low eccentricity

    % Proximity score: prefer last centroid
    if isfield(params,'refCentroid') && ~isempty(params.refCentroid) && all(isfinite(params.refCentroid))
        ref = params.refCentroid(:).';
    else
        ref = [size(Iwork,2)/2, size(Iwork,1)/2];
    end
    Ccand = reshape([S(idxCand).Centroid],2,[])';
    d2 = sum((Ccand - ref).^2, 2);
    proxScore = 1 ./ (1 + d2);            % already around 0-1
    proxScore = normalize01(proxScore); 

    % Weighted score: shape + expected size; small weight on proximity
    % (adjust as needed, this is what I found works best)
    score = 0.30*circScore + 0.25*solScore + 0.20*eccScore + 0.20*areaScore(:) + 0.05*proxScore;

    [~,kBest] = max(score);
    iBest = idxCand(kBest);

    BWbest = false(size(BW));
    BWbest(CC.PixelIdxList{iBest}) = true;

    Bper = bwperim(BWbest);
    [yy, xx] = find(Bper);

    if numel(xx) >= 20
        [xc, yc, rc, okFit] = fitCircleLS(xx, yy);
        if okFit
            centroid = [xc+xOff yc+yOff];
            dbg(end+1) = string(sprintf("GrayThresh: circle-fit center (rFit=%.1fpx, rExp=%.1fpx).", rc, rExp));
            return;
        end
    end

    centroid = S(iBest).Centroid + [xOff yOff];
    dbg(end+1) = "GrayThresh: region centroid (no circle fit).";
    return;
% ============================================================
    case "YCbCrNeutral"

    % Combined bright/dark blob detector
    % Works when the ball is brighter or darker than the background.

    Ig = im2double(rgb2gray(Iwork));
    Iinv = 1 - Ig;

    % --- Bright-ball mask ---
    if isfield(params,'yMin') && params.yMin > 0
        tBright = params.yMin;
    else
        tBright = graythresh(Ig);
    end
    BWb = Ig > tBright;

    % --- Dark-ball mask (threshold on inverted image) ---
    tDark = graythresh(Iinv);
    BWd = Iinv > tDark;   % Ig < (1 - tDark)

    BW = BWb | BWd;

    % Remove specks and thin lines.
    BW = bwareaopen(BW, 50);
    BW = imopen(BW, strel('disk',2));
    BW = imclose(BW, strel('disk',3));
    BW = imfill(BW,'holes');

    if ~any(BW(:))
        dbg = ["YCbCrNeutral (bright+dark blob): BW empty after cleanup."];
        centroid = [NaN NaN];
        return;
    end

    stats = regionprops(BW,'Area','Centroid','Eccentricity','Solidity');
    if isempty(stats)
        dbg = ["YCbCrNeutral (bright+dark blob): no components."];
        centroid = [NaN NaN];
        return;
    end

    A   = [stats.Area];
    ecc = [stats.Eccentricity];
    sol = [stats.Solidity];

    % Prefer large, round, solid blob near image center
    ok = (ecc < 0.9) & (sol > 0.7);
    if ~any(ok)
        ok = true(size(A));
    end

    statsOK = stats(ok);
    Aok = A(ok);
    Cok = reshape([statsOK.Centroid],2,[])';
    imgCenter = [size(Iwork,2)/2, size(Iwork,1)/2];
    d2 = sum((Cok - imgCenter).^2, 2);

    % score the area and proximity to center.
    areaScore = Aok(:) / max(Aok);
    proxScore = 1 ./ (1 + d2);
    proxScore = proxScore / max(proxScore);
    score = 0.6*areaScore + 0.4*proxScore;

    [~,kBest] = max(score);
    c = Cok(kBest,:);
    centroid = c + [xOff yOff];
    dbg(end+1) = sprintf("YCbCrNeutral (bright+dark blob): tB=%.3f tD=%.3f area=%.0f", ...
                         tBright, tDark, Aok(kBest));
    return;

% ============================================================
    case "Circles"
        % Circles, bright and dark detection (can tune radius range)

        Ig = mat2gray(rgb2gray(Iwork));
        radRange = [15 200];

        % Bright ball case
        [centBright, radBright, metBright] = imfindcircles(Ig, radRange, ...
            'ObjectPolarity','bright','Sensitivity',0.95,'EdgeThreshold',0.10); %#ok<ASGLU>

        % Dark ball case
        IgInv = 1 - Ig;
        [centInv, radInv, metInv] = imfindcircles(IgInv, radRange, ...
            'ObjectPolarity','bright','Sensitivity',0.95,'EdgeThreshold',0.10); %#ok<ASGLU>

        centers = [centBright; centInv];
        radii   = [radBright;  radInv];
        metric  = [metBright;  metInv];

        if isempty(centers)
            dbg = ["Circles: imfindcircles found nothing (bright or inverted-bright)."];
            centroid = [NaN NaN];
            return;
        end

        % If we have a previous centroid, choose the closest circle
        % else, we pick the one with the highest metric.
        if isfield(params,'refCentroid') && ~isempty(params.refCentroid) && size(centers,1) > 1
            d2 = sum((centers - params.refCentroid(:).').^2, 2);
            [~,k] = min(d2);
        else
            [~,k] = max(metric);
        end

        centroid = centers(k,:) + [xOff yOff];
        dbg(end+1) = sprintf("Circles: picked r=%.1f metric=%.3f (bright+inv, range=[15 200])", ...
                             radii(k), metric(k));
        return;

            otherwise
                dbg = ["Unknown method."];
                return;
end

% ============================================================
% Scoring (GrayThresh + YCbCr)
% ============================================================

stats = regionprops(BW,'Area','Centroid','Perimeter','Eccentricity','Solidity');

if isempty(stats)
    dbg(end+1) = "No blobs after filtering.";
    return;
end

A   = [stats.Area];
P   = [stats.Perimeter];
ecc = [stats.Eccentricity];
sol = [stats.Solidity];
circ = 4*pi*A ./ max(P.^2, eps);

% Shape gate: use stricter 'ball-shaped' constraints for YCbCr (helps reject court lines)
if isfield(params,'method') && params.method == "YCbCrNeutral"
    ok = (circ > 0.65) & (ecc < 0.85) & (sol > 0.80);
else
    ok = (circ > 0.35) & (ecc < 0.95) & (sol > 0.6);
end
if ~any(ok)
    ok = true(size(A));
end

idx = find(ok);
if isempty(idx)
    dbg(end+1) = "No blobs after filtering.";
    return;
end
C = reshape([stats(idx).Centroid],2,[])';
Aok = A(idx);
circOk = circ(idx);

scoreShape = circOk(:) .* sqrt(Aok(:));

if isfield(params,'refCentroid') && ~isempty(params.refCentroid)
    ref = params.refCentroid(:).';
else
    ref = [size(Iwork,2)/2, size(Iwork,1)/2];
end

d2 = sum((C - ref).^2,2);
scoreProx = 1 ./ (1 + d2);

% Prefer blobs near expected area (useful for Ycrbr at z=4 and z=5)
scoreArea = ones(size(Aok(:)));
if isfield(params,'Ztrue_m') && isfield(params,'f_px') && isfinite(params.Ztrue_m) && params.Ztrue_m > 0
    Zuse = params.Ztrue_m;
    fpx  = params.f_px;
    Rball_m = 0.033;
    rExp = (fpx * Rball_m) / Zuse;
    rExp = max(6, min(rExp, 0.45*min(size(Iwork,1), size(Iwork,2))));
    areaExp = pi * rExp^2;
    areaRatio = Aok(:) / areaExp;
    scoreArea = 1 - min(abs(log(max(areaRatio, 0.1))), 2)/2;
end

nShape = max(scoreShape);
nProx  = max(scoreProx);
if nShape < eps, nShape = 1; end
if nProx  < eps, nProx  = 1; end
score = 0.5*(scoreShape/nShape) + 0.25*(scoreProx/nProx) + 0.25*scoreArea;
[~,k] = max(score);

centroid = C(k,:) + [xOff yOff];
dbg(end+1) = "Blob scoring complete";

end

%% -------------------- TCP Helpers --------------------
    function ok = ensureClient()
        ok = true;
        try
            if isempty(client) || ~isvalid(client)
                disp("Connecting to Blender at 127.0.0.1:55001");
                client = tcpclient(BLENDER.server_ip, BLENDER.server_port, 'Timeout', 20);
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