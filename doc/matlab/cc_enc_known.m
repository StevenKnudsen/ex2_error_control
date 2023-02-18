% CCSDS G1 = 121 (0x79) G2 = 91 (0x5B)
% Voyager G1 = 109 0o155 (0x6D) G2 = 79 0o117 (0x4F)
% CCSDS G1 = 121 0o171 (0x79) G2 = 91 0o133 (0x5B)
trellis = poly2trellis(7,[171,133]);
tbl = 32;
rate = 1/2;

dataIn = 1:255;

data = [];
for n = 1:length(dataIn)
    d = cast(dataIn(n),"uint8");
    for b = 1:8
%        fprintf("0x%02x\n",d)
        if bitand(d,0x80) > 0
           fprintf("1");
            data = [data,1];
        else
           fprintf("0");
            data = [data,0];
        end
    d = bitshift(d,1);
    end
    fprintf("\n");
end


% Convolutionally encode the data
dataEnc = convenc(data,trellis);

fid = fopen("cc_enc_known.hpp","w");
fprintf(fid,"std::vector<uint8_t> message = {\n");
for n = 0:8:(length(data)-8)
    for m = 1:8
        fprintf(fid,"%d,",data(n+m));
    end
    fprintf(fid,"\n");
end
fprintf(fid,"}\n");
fprintf(fid,"std::vector<uint8_t> codeword = {\n");
for n = 0:8:(length(dataEnc)-8)
    for m = 1:8
        fprintf(fid,"%d,",dataEnc(n+m));
    end
    fprintf(fid,"\n");
end
fprintf(fid,"}\n");
fclose (fid);
