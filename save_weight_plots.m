function save_weight_plots(path, log, type, time)

    figure;
    t = tiledlayout(3, 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');
    title(t, strcat(path(1:end-1), " Veriseti için ", type, " Ortalama Metodu Ağırlıkları"), ...
          'Interpreter', 'none', 'FontName', 'Arial Unicode MS'); % Explicit font and interpreter

    % X
    nexttile;
    plot(time, log(1,:), 'DisplayName', 'X yönündeki LiDAR Ağırlığı');
    grid on;
    xlabel('Zaman (s)', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');
    ylabel('Ağırlık Değeri', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');
    xlim([0, time(end)]); 
    ylim([0, max(log(:))]); % Adjust ylim to use max(log(:))
    title('X yönündeki LiDAR Ağırlığı', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');

    % Y
    nexttile;
    plot(time, log(2,:), 'DisplayName', 'Y yönündeki LiDAR Ağırlığı');
    grid on;
    xlabel('Zaman (s)', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');
    ylabel('Ağırlık Değeri', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');
    xlim([0, time(end)]); 
    ylim([0, max(log(:))]);
    title('Y yönündeki LiDAR Ağırlığı', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');

    % Z
    nexttile;
    plot(time, log(3,:), 'DisplayName', 'Z yönündeki LiDAR Ağırlığı');
    grid on;
    xlabel('Zaman (s)', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');
    ylabel('Ağırlık Değeri', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');
    xlim([0, time(end)]); 
    ylim([0, max(log(:))]);
    title('Z yönündeki LiDAR Ağırlığı', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');

    % Exporting the figure as a PDF using 'print' for full font embedding
    set(gcf, 'PaperPositionMode', 'auto'); % Ensure the saved figure matches the on-screen display
    print(gcf, strcat(path, type, '_weights.pdf'), '-dpdf', '-bestfit'); % Save as PDF with embedded fonts
end
