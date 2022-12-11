clear all

syms Ia Iirr_ref G Gref alpha_T Ta Tref Io_ref Eg_ref k Eg q Va Rs_ref Ns n_ref Rp_ref

Imax = 0.8;
Imin = 0.01;
Tmax = 85;
Tmin = 15;
Vmax = 13.5;
Vmin = 0.01;

for i = 1:2000

    Ia = (Imax-Imin)*rand + Imin;
    Ta = (Tmax-Tmin)*rand + Tmin;
    Va = (Vmax-Vmin)*rand + Vmin;
    
    Gref = 1000;
    alpha_T = 0.046;
    beta_T = -0.0118;
    Tref = 25;
    T_kelv = Ta + 273.15;
    k = 1.3806 * (10^-23);
    Eg = 1.16 - 7.02 * (10^-4) * ((T_kelv^2)/T_kelv - 1108);
    Eg_ref = 1.12;
    q = 1.602 * (10^-19);
    n = 1.3;
    Ns = 24;
    %Ns = 12;
    Np = 2;
    %Np = 1;
    Ns_module = 2;
    %Ns_module = 1;
    Np_module = 2;
    %Np_module = 1;
    Rp_ref = 240;
    Rs_ref = 0.49739;
    Io_ref = 3.49*(10^-11);
    Iirr_ref = 0.400005;
    Vmp_ref = 6 * Ns_module;
    Imp_ref = 0.35 * Np_module;
    Voc_ref = 6.6 * Ns_module;
    Isc_ref = 0.4 * Np_module;
    
    
    Iirr = Iirr_ref * (G / Gref) * (1+ alpha_T * (Ta - Tref));
    Iirr_prime = Iirr * Np;
    Io = Io_ref*((Ta/Tref)^3)*exp((Eg_ref / (k * Tref))-(Eg/(k*Ta)));
    Io_prime = Io * Np;
    Rp = Rp_ref*(G/Gref);
    Rp_prime = Rp * (Ns/Np);
    Rs = Rs_ref;
    Rs_prime = Rs * (Ns/Np);
    expon = exp(((q*(Va+Ia*Rs_prime))/(Ns*n*k*Ta)));
    
    eqn = ((Iirr_prime - (Io_prime * (expon-1)) - ((Va + Ia*Rs_prime)/(Rp_prime))) == Ia);
    
    try
        G_solv = vpasolve(eqn,G);
        G_est(i,1) = double(abs(G_solv(2,1)));
        IMP(i) = Imp_ref * (G_est(i)/Gref)*(1+(alpha_T*(Ta-Tref)));
        VMP(i) = Vmp_ref + (beta_T*(Ta-Tref));
        PMP(i) = VMP(i) * IMP(i)
    
        input(i,:) = [Va Ia Ta];
        output_v(i,1) = VMP(i);
        output_i(i,1) = IMP(i);
        output_p(i,1) = PMP(i);
    catch

    end
    
end

