function helperShowError(errors_translation, errors_rotation, errors_reprojection, names)
%helperShowError Helper function to display calibration errors
%
% This is an example helper function that is subject to change or removal in
% future releases.

% Copyright 2019 The MathWorks, Inc.


% Create figure
figureH = figure('Visible','off','Position',[0, 0, 1200, 640],...
    'Name','Error Plots');

% Create panel for translation error plot
panel1 = uipanel('Parent',figureH,'Position',[0.04,0.58,0.42,0.38],...
    'Title','Translation Error','FontSize',15,'TitlePosition','centertop');
axes1 = axes('Parent',panel1,'Position',[0.1 0.1 0.8 0.8],'NextPlot','add');
axes1.Toolbar.Visible = 'off';
axis(axes1,'tight');
disableDefaultInteractivity(axes1)

% Create panel for rotational error plot
panel2 = uipanel('Parent',figureH,'Position',[0.55,0.58,0.40,0.38],...
    'Title','Rotation Error','FontSize',15,'TitlePosition','centertop');
axes2 = axes('Parent',panel2,'Position',[0.1 0.1 0.8 0.8],'NextPlot','add');
axes2.Toolbar.Visible = 'off';
axis(axes2,'tight');
disableDefaultInteractivity(axes2)

% Create panel for reprojection error plot
panel3 = uipanel('Parent',figureH,'Position',[0.25,0.03,0.40,0.38],...
    'Title','Reprojection Error','FontSize',15,'TitlePosition','centertop');
axes3 = axes('Parent',panel3,'Position',[0.1 0.1 0.8 0.8],'NextPlot','add');
axes3.Toolbar.Visible = 'off';
axis(axes3,'tight');
disableDefaultInteractivity(axes3)

set(figureH,'Visible','on');
bar(axes1,errors_translation);
xticks(axes1, 1:size(names, 1));
xticklabels(axes1, replace(names, "_", "\_"));
line1H = yline(axes1,mean(errors_translation),'--','Color','r');
legend(line1H,strcat('Overall Mean Translation Error: ', num2str(mean(errors_translation)),' in m'),'Location','southeast');

bar(axes2,errors_rotation);
xticks(axes2, 1:size(names, 1));
xticklabels(axes2, replace(names, "_", "\_"));
line2H=yline(axes2,mean(errors_rotation),'--','Color','r');
legend(line2H,strcat('Overall Mean Rotation Error: ', num2str(mean(errors_rotation)),' in deg'),'Location','southeast');

bar(axes3,errors_reprojection);
xticks(axes3, 1:size(names, 1));
xticklabels(axes3, replace(names, "_", "\_"));
line3H=yline(axes3,mean(errors_reprojection),'--','Color','r');
legend(line3H,strcat('Overall Mean Reprojection Error: ', num2str(mean(errors_reprojection)),' in pixel'),'Location','southeast');

end