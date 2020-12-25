function xnew = lowpass_fft(x, Fs, cutoff)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    y = fft(x);     % FFT  

    n = length(x);                         
    fshift = (-n/2:n/2-1)*(Fs/n);
    yshift = fftshift(y);
    % Remove high frequency components
    I = fshift<=cutoff & fshift>=-cutoff;
    yshift(~I) = 0;
    % Recover the signal from freq domain
    ynew = ifftshift(yshift);
    xnew = real(ifft(ynew));
end

