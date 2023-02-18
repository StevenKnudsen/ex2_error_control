format long;

bpskModulator = comm.BPSKModulator;
bpskDemodulator = comm.BPSKDemodulator;

errorRate = comm.ErrorRate;

rng default
M = 2;                 % Modulation order
k = log2(M);           % Bits per symbol
EbNoVec = (4:10)';    % Eb/No values (dB)
numSymPerFrame = 1000; % Number of QAM symbols per frame

rate = 1;

berEstSoft = zeros(size(EbNoVec)); 
berEstHard = zeros(size(EbNoVec));

for n = 1:length(EbNoVec)
    % Convert Eb/No to SNR
    snrdB = EbNoVec(n) + 10*log10(k*rate);
    % Noise variance calculation for unity average signal power
    noiseVar = 10.^(-snrdB/10);
    % Reset the error and bit counters
    numErrsSoft = 0;
    numErrsHard = 0;
    numBits = 0;
%    [numErrsSoft,numErrsHard,numBits] = deal(0);
    
    while numErrsSoft < 100 && numBits < 1e7
        % Generate binary data and convert to symbols
        dataIn = randi([0 1],numSymPerFrame*k,1);

        txSig = bpskModulator(dataIn);        % Modulate
        rxSig = awgn(txSig,snrdB,'measured'); % Pass through AWGN
        rxDataHard = bpskDemodulator(rxSig);  % Demodulate
        rxDataSoft = bpskDemodulator(rxSig);

        numErrsH = biterr(dataIn,rxDataHard); % Collect error stats
        numErrsS = biterr(dataIn,rxDataSoft); % Collect error stats

        % Increment the error and bit counters
        numErrsHard = numErrsHard + numErrsH;
        numErrsSoft = numErrsSoft + numErrsS;
        numBits = numBits + numSymPerFrame*k;
    
%    fprintf('Error rate = %e\nNumber of errors = %d\n', ...
%        errorStats(1), errorStats(2));
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
