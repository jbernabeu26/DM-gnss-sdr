%% BeiDou B2a Pilot Signal Secondary Code Generation
%---Generates the B2a Pilot secondary codes for tracking implementations

clear;
close all;
N = 1021;

BEIDOU_B2ap_SECONDARY_TRUNCATION_POINT = ...
    {
    	138, 570, 351,   77, 885, 247, 413, 180,   3,  26, ...
		 17, 172,  30, 1008, 646, 158, 170,  99,  53, 179, ...
		925, 114,  10,  584,  60,   3, 684, 263, 545,  22, ...
    	546, 190, 303,  234,  38, 822,  57, 668, 697,  93, ...
		 18,  66, 318,  133,  98,  70, 132,  26, 354,  58, ...
		 41, 182, 944,  205,  23,   1, 792, 641,  83,   7, ...
		111,  96,  92
    };

BEIDOU_B2ap_SECONDARY_PHASE_DIFFERENCE = ...
    {
    	123,  55,  40, 139,  31, 175, 350, 450, 478,   8, ...
		 73,  97, 213, 407, 476,   4,  15,  47, 163, 280, ...
		322, 353, 375, 510, 332,   7,  13,  16,  18,  25, ...
		 50,  81, 118, 127, 132, 134, 164, 177, 208, 249, ...
		276, 349, 439, 477, 498,  88, 155, 330,   3,  21, ...
		 84, 111, 128, 153, 197, 199, 214, 256, 265, 291, ...
		324, 326, 340
    };

bds_b2a_sec_pilot = strings(63,1);

%% Pilot secondary code generation
for i=1:63
    %--- Get weil code with w phase difference
    w = BEIDOU_B2ap_SECONDARY_PHASE_DIFFERENCE{i};
    p = BEIDOU_B2ap_SECONDARY_TRUNCATION_POINT{i};
    
    %--- Generate truncated weil code
    sec_pilot_seq = truncated_weil(N, w, p);
    
    %--- Secondary code string
    bds_b2a_sec_pilot(i) = sprintf('%d',sec_pilot_seq(1:100));
    
    %--- Check values against ICD
    first_octal = fliplr(sec_pilot_seq(1:24));
    last_octal  = fliplr(sec_pilot_seq(77:100));
    fprintf('\nPRN: %d, First 24 Chips (Octal): %s First 24 Chips (Octal): %s \n', ...
        i, dec2base(bi2de(first_octal),8), dec2base(bi2de(last_octal),8));
end

%% Utility functions
function W = truncated_weil(N, w, p)

W=zeros(1,N);
count = 1;
for n=0: N-1
    k=mod(n+p-1, N);
    W(count) = xor(leg(k,N), leg(mod(k+w, N),N));

    count = count + 1;
end
end

function[W] = weil (p, w)

% WEIL Weil codes.
%    [w,W] = WEIL(P) generates a binary Weil code w and Weil code
%    sequence W.  P specifies the length of the code; it must be
%    a prime number.
%
%    WEIL(...,CODENUM) returns the NUM-th Weil code.  If missing,
%    default is CODENUM = 1.

W=zeros(1,p);
count = 1;
for k = 0:p-1
    
    W(count) = xor(leg(k,p), leg(mod(k+w, p),p));

    count = count + 1;
end

end

function L = leg(k,p)
% Compute the Legendre sequence

if (k == 0)
    L = 1;
else
    if (mod(k,p) == 0)
        L = NaN;   % p divides i (doesn't happen for primes)
    else
        squaremodp = 0;
        z = 1;
        while (z <= (p-1)/2)
            if (mod(z*z,p) == k)
                squaremodp = 1;
                break;
            end
            z = z + 1;
        end
        
        if (squaremodp)
            L = 0;   % i is a square(mod p)
        else
            L = 1;   % i is not a square(mod p)
        end
    end
end
end