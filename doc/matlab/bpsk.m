format long;

bpskModulator = comm.BPSKModulator;
bpskDemodulatorSoft = comm.BPSKDemodulator;
bpskDemodulatorSoft.DecisionMethod='Approximate log-likelihood ratio';
bpskDemodulatorHard = comm.BPSKDemodulator;

% CCSDS G1 = 121 (0x79) G2 = 91 (0x5B)
% Voyager G1 = 109 0o155 (0x6D) G2 = 79 0o117 (0x4F)
% CCSDS G1 = 121 0o171 (0x79) G2 = 91 0o133 (0x5B)
trellis = poly2trellis(7,[171,133]);
tbl = 32;
rate = 1/2;

errorRate = comm.ErrorRate;

rng default
M = 2;                 % Modulation order
k = log2(M);           % Bits per symbol
EbNoVec = (-2:6)';    % Eb/No values (dB)
numSymPerFrame = 1000; % Number of QAM symbols per frame

berEstSoft = zeros(size(EbNoVec)); 
berEstHard = zeros(size(EbNoVec));

for n = 1:length(EbNoVec)
    % Convert Eb/No to SNR.
    snrdB = EbNoVec(n) + 10*log10(k*rate);
    % Noise variance calculation for unity average signal power
    noiseVar = 10.^(-snrdB/10);
    % Reset the error and bit counters
    numErrsSoft = 0;
    numErrsHard = 0;
    numBits = 0;
    
    while numErrsSoft < 100 && numBits < 1e7
        % Generate binary data and convert to symbols
        dataIn = randi([0 1],numSymPerFrame*k,1);

        % Convolutionally encode the data
        dataEnc = convenc(dataIn,trellis);

        txSig = bpskModulator(dataEnc);        % Modulate

        rxSig = awgn(txSig,snrdB,'measured'); % Pass through AWGN

        % Demodulate
        rxDataHard = bpskDemodulatorHard(rxSig);  
        rxDataSoft = bpskDemodulatorSoft(rxSig);

                % Viterbi decode the demodulated data
        dataHard = vitdec(rxDataHard,trellis,tbl,'cont','hard');
        dataSoft = vitdec(rxDataSoft,trellis,tbl,'cont','unquant');

        % Calculate the number of bit errors in the frame. 
        % Adjust for the decoding delay, which is equal to 
        % the traceback depth.
        numErrsH = ...
            biterr(dataIn(1:end-tbl),dataHard(tbl+1:end));
        numErrsS = ...
            biterr(dataIn(1:end-tbl),dataSoft(tbl+1:end));

        % Increment the error and bit counters
        numErrsHard = numErrsHard + numErrsH;
        numErrsSoft = numErrsSoft + numErrsS;
        numBits = numBits + numSymPerFrame*k;
    
    end

        % Estimate the BER for both methods
    berEstSoft(n) = numErrsSoft/numBits;
    berEstHard(n) = numErrsHard/numBits;

end

semilogy(EbNoVec,[berEstSoft berEstHard],'-*')
hold on
semilogy(EbNoVec,berawgn(EbNoVec,'psk',M,'nondiff'))
legend('Soft','Hard','Uncoded','location','best')
grid
xlabel('Eb/No (dB)')
ylabel('Bit Error Rate')
