function fig2pdf(figHandle, file_path)
    if ~contains(file_path, '.pdf')
        file_path = strcat(file_path, '.pdf');
    end
    % MATLAB 2020a introduced this nice function 'exportgraphics
    if contains(version, 'R2020')
        exportgraphics(figHandle, file_path,'ContentType','vector');
    else
        print(figHandle, file_path, '-dpdf', '-r0','-fillpage');
        fprintf('PDF file is not tightly trimed, need to do it manually.\n');
    end
    
end