#ifndef GNSS_SDR_BEIDOU_B1C_SIGNAL_PROCESSING_H_
#define GNSS_SDR_BEIDOU_B1C_SIGNAL_PROCESSING_H_

#include <complex>
#include <cstdint>

#if HAS_SPAN
#include <span>
namespace gsl = std;
#else
#include "gsl/gsl"
#endif

//! Generates BeiDou B1c Data Primary codes for the desired SV ID
void make_b1cd(gsl::span<int32_t> _dest, int32_t prn);


//! Generate a float version of the B1C Data Primary code
void beidou_b1cd_code_gen_float(gsl::span<float> _dest, uint32_t _prn);


//! Generate a complex version of the B1C Data Primary code
void beidou_b1cd_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn);


//! Generates complex BEIDOU B1C Data Primary code for the desired SV ID and sampled to specific sampling frequency
void beidou_b1cd_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs);


//! Generate the B1C Pilot Primary code
void make_b1cp(gsl::span<int32_t> _dest, int32_t prn);


//! Generate a float version of the B1C Pilot Primary code
void beidou_b1cp_code_gen_float(gsl::span<float> _dest, uint32_t _prn);


//! Generate a complex version of the B1C Pilot Primary code
void beidou_b1cp_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn);


//! Generates complex BEIDOU B1C Primary pilot code for the desired SV ID and sampled to specific sampling frequency
void beidou_b1cp_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs);


//! Generate a version of the B1C Pilot code with the secondary pilot code included
void make_b1cp_secondary(gsl::span<int32_t> _dest, int32_t prn);


//! Generate a complex version of the B1C pilot code with the secondary pilot code
void beidou_b1cp_code_gen_complex_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn);


//! Generates a float version of the B1C pilot primary code
void beidou_b1cp_code_gen_float_secondary(gsl::span<float> _dest, uint32_t _prn);


//! Generates complex BEIDOU B1C pilot code for the desired SV ID and sampled to specific sampling frequency with the secondary code implemented
void beidou_b1cp_code_gen_complex_sampled_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs);


//! Generates complex BEIDOU B1C data+pilot code for the desired SV ID and sampled to specific sampling frequency
void beidou_b1c_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs);



#endif /* GNSS_SDR_BEIDOU_B1C_SIGNAL_PROCESSING_H_ */
