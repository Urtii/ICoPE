function save_spectrum_plots(path, log, type, time)
    
    % Define frequency range for analysis
    f = linspace(0, 5, 500); % Frequency range (Hz)
    
    % Compute Lomb-Scargle Periodogram
    [pxx1, f1] = plomb(log(1,:), time, f);
    [pxx2, f2] = plomb(log(2,:), time, f);
    [pxx3, f3] = plomb(log(3,:), time, f);

    figure;
    t = tiledlayout(3, 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');
    title(t, strcat(path(1:end-1), " Veriseti için ", type, " Ortalama Metodu Spektrumu"), ...
          'Interpreter', 'none', 'FontName', 'Arial Unicode MS'); % Explicit font and interpreter

    % X
    nexttile;
    plot(f1, pxx1, 'DisplayName', 'X yönündeki LiDAR Ağırlık Spektrumu');
    grid on;
    xlabel('Frekans (Hz)', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');
    ylabel('Güç', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');
    % xlim([0, time(end)]); 
    ylim([0, max([pxx1;pxx2;pxx3])]); % Adjust ylim to use max(log(:))
    title('X yönündeki LiDAR Ağırlık Spektrumu', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');

    % Y
    nexttile;
    plot(f2, pxx2, 'DisplayName', 'X yönündeki LiDAR Ağırlık Spektrumu');
    grid on;
    xlabel('Frekans (Hz)', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');
    ylabel('Güç', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');
    % xlim([0, time(end)]); 
    ylim([0, max([pxx1;pxx2;pxx3])]); % Adjust ylim to use max(log(:))
    title('X yönündeki LiDAR Ağırlık Spektrumu', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');

    % Z
    nexttile;
    plot(f3, pxx3, 'DisplayName', 'X yönündeki LiDAR Ağırlık Spektrumu');
    grid on;
    xlabel('Frekans (Hz)', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');
    ylabel('Güç', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');
    % xlim([0, time(end)]); 
    ylim([0, max([pxx1;pxx2;pxx3])]); % Adjust ylim to use max(log(:))
    title('X yönündeki LiDAR Ağırlık Spektrumu', 'Interpreter', 'none', 'FontName', 'Arial Unicode MS');

    % Exporting the figure as a PDF using 'print' for full font embedding
    set(gcf, 'PaperPositionMode', 'auto'); % Ensure the saved figure matches the on-screen display
    print(gcf, strcat(path, type, '_spectrum.pdf'), '-dpdf', '-bestfit'); % Save as PDF with embedded fonts
end
