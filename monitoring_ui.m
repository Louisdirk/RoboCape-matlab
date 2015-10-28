function monitoring_ui(Fig, Time, Img, Var1, Var1Name, Var2, Var2Name, Var3, Var3Name, VarLim)
    figure(Fig);

    subplot(3, 5, [1:3 6:8]);
    subimage(Img);

    subplot(3, 5, [4:5]); hold on;
    plot_topic(Time, Var1, Var1Name, VarLim);

    subplot(3, 5, [9:10]); hold on;
    plot_topic(Time, Var2, Var2Name, VarLim);

    subplot(3, 5, [14:15]); hold on;
    plot_topic(Time, Var3, Var3Name, VarLim);

    Fig.Visible = 'on';
end
