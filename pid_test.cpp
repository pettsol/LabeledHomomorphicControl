#include "LabeledHE/labeled_he.h"
#include "labeled_he_control.h"
#include "joye_libert_journal/joye_libert.h"

#include <gmp.h>
#include <iostream>
#include <chrono>
#include <unistd.h>

int main()
{
	std::cout << "Hello World\n";

	// Declare control parameters
	// NB! Gain in proportional and
	// integral part must be the same. I.e.,
	// if we scale the timestamp, we must compensate by
	// scaling Kp.
	mpz_t kp, ki, kd;
	mpz_init_set_ui(kp, 10000); // Kp = 10, scaled by 1000
	mpz_init_set_ui(ki, 80); // Ki = 8, scaled by 10
	mpz_init_set_ui(kd, 1); // Kd = 0.1, scaled by 10

	// Declare cryptographic parameters
	mpz_t N, y, p, ptspace, half_ptspace;
	mpz_init(N);
	mpz_init(y);
	mpz_init(p);
	mpz_init(ptspace);
	mpz_init(half_ptspace);

	gmp_randstate_t state;
	gmp_randinit_mt(state);

	uint32_t keysize = 2048;
	uint32_t msgsize = 32;

	mpz_ui_pow_ui(ptspace, 2, msgsize);
	mpz_ui_pow_ui(half_ptspace, 2, msgsize-1);
	
	std::cout << "Generating appropriate keys - this may take a while.\n";
	he_keygen(N, y, p, msgsize, keysize);

	hc128_state hc_cs;
        uint8_t hc_key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                              0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15};
        uint8_t hc_iv[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	hc128_initialize(&hc_cs, hc_key, hc_iv);

	// HE Encryption Labels
	mpz_t kp_label, ki_label, kd_label, e_label, time_label, time_inverse_label;
	mpz_init_set_ui(kp_label, 1);
	mpz_init_set_ui(ki_label, 4);
	mpz_init_set_ui(kd_label, 6);
	mpz_init_set_ui(e_label, 2);
	mpz_init_set_ui(time_label, 3);
	mpz_init_set_ui(time_inverse_label, 5);

	// HE Encryption output
	mpz_t b_kp, b_ki, b_kd, b_e, b_t, b_t_inv;
	mpz_init(b_kp);
	mpz_init(b_ki);
	mpz_init(b_kd);
	mpz_init(b_e);
	mpz_init(b_t);
	mpz_init(b_t_inv);

	// HE Ciphertext output
	he_ct c_kp, c_ki, c_kd, c_e, c_t, c_t_inv;

	// Encrypted Control Output
	mpz_t c_u, c_u_test;
	mpz_init(c_u);
	mpz_init(c_u_test);

	// Encrypt the control parameters
	he_encrypt(&c_kp, state, &hc_cs, b_kp, kp, y, N, kp_label, msgsize);
	he_encrypt(&c_ki, state, &hc_cs, b_ki, ki, y, N, ki_label, msgsize);
	he_encrypt(&c_kd, state, &hc_cs, b_kd, kd, y, N, kd_label, msgsize);

	// Initialize Homomorphic Proportional-Integral Controller
	Labeled_he_pid_control he_pid_controller(&c_kp, &c_ki, &c_kd, y, N, msgsize);

	// Decrypted Control Output
	mpf_t u;
	mpf_init(u);

	// ASSUME STATE BETWEEN -1 and 1
	double desirable_state = 10;
	double current_state = 0;
	double error;
	mpz_t error_bar, timestep_bar, timestep_bar_inv;
	mpz_init(error_bar);
	mpz_init(timestep_bar);
	mpz_init(timestep_bar_inv);

	// Integral component is stateful:
	mpz_t b_int;
	mpz_init(b_int);
	int is_init = 0;
	// Get initial timepoint, map to b_t_prev
	auto prev_tp = std::chrono::system_clock::now();
	mpz_t b_t_prev;
	mpz_init(b_t_prev);

	mpz_t b_e_prev, b_delta_e;
	mpz_init_set_ui(b_e_prev, 0);
	mpz_init(b_delta_e);

	for (int i = 0; i < 1000; i++)
	{
		// Switch sign of desired state every 250 samples
		if ( (i != 0) && (i % 250 == 0) )
		{
			std::cout << "\n\nFLIPPING DESIRABLE STATE\n\n";
			desirable_state = -desirable_state;
		}

		// Sleep for 1 second
		usleep(100000);
		// Get current time, and find delta t:
		auto sample_tp = std::chrono::system_clock::now();
		std::chrono::duration<float> ts = sample_tp - prev_tp;
		float timestep = ts.count();
		prev_tp = sample_tp;

		// Map timestep to non-negative integer! Scaled by 100
		float timestep_inv = (1/timestep) * 100;
		timestep = timestep*100;//*100000;// * 10;
		mpz_set_d(timestep_bar, timestep);
		mpz_set_d(timestep_bar_inv, timestep_inv);
		
		he_encrypt(&c_t, state, &hc_cs, b_t, timestep_bar, y, N, time_label, msgsize);
		he_encrypt(&c_t_inv, state, &hc_cs, b_t_inv, timestep_bar_inv, y, N, time_inverse_label, msgsize);

		// Find state error
		error = desirable_state - current_state;

		// Map error to non-negative integer! Scaled by 100 000
		error = error * 100000;
		mpz_set_d(error_bar, error);
		mpz_mod(error_bar, error_bar, ptspace);
		
		// Encrypt the error
		he_encrypt(&c_e, state, &hc_cs, b_e, error_bar, y, N, e_label, msgsize);

		if(i == 0)
		{
			mpz_set(b_e_prev, b_e);
		}

		// Compute betait, betaie, betate
		mpz_t betait, betaie, betate;
		mpz_init(betait);
		mpz_init(betaie);
		mpz_init(betate);
		mpz_mul(betait, b_ki, b_t);
		mpz_mod(betait, betait, ptspace);
		mpz_mul(betaie, b_ki, b_e);
		mpz_mod(betaie, betaie, ptspace);
		mpz_mul(betate, b_t, b_e);
		mpz_mod(betate, betate, ptspace);
		joye_libert_encrypt(betait, state, betait, y, N, msgsize);
		joye_libert_encrypt(betaie, state, betaie, y, N, msgsize);
		joye_libert_encrypt(betate, state, betate, y, N, msgsize);

		// Compute betadt_inv, betade, betat_inve
		mpz_sub(b_delta_e, b_e, b_e_prev);
		//
		mpz_t betadt_inv, betade, betat_inve;
		mpz_init(betadt_inv);
		mpz_init(betade);
		mpz_init(betat_inve);
		mpz_mul(betadt_inv, b_kd, b_t_inv);
		mpz_mod(betadt_inv, betadt_inv, ptspace);
		mpz_mul(betade, b_kd, b_delta_e);
		mpz_mod(betade, betade, ptspace);
		mpz_mul(betat_inve, b_t_inv, b_delta_e);
		mpz_mod(betat_inve, betat_inve, ptspace);
		joye_libert_encrypt(betadt_inv, state, betadt_inv, y, N, msgsize);
		joye_libert_encrypt(betade, state, betade, y, N, msgsize);
		joye_libert_encrypt(betat_inve, state, betat_inve, y, N, msgsize);
		
		he_pid_controller.iterate(c_u, &c_t, &c_t_inv, &c_e, betait, betaie, betate, 
						betadt_inv, betade, betat_inve);

		mpz_t u_bar, u_bar_test, b_u, b_prop;
		mpz_init(u_bar);;
		mpz_init(b_u);
		mpz_init(b_prop);

		// Compute the integral part
		mpz_t b_tmp_int;
		mpz_init(b_tmp_int);
		mpz_mul(b_tmp_int, b_t, b_ki);
		mpz_mod(b_tmp_int, b_tmp_int, ptspace);
		mpz_mul(b_tmp_int, b_tmp_int, b_e);
		mpz_mod(b_tmp_int, b_tmp_int, ptspace);
		// Add new element to sum
		mpz_add(b_int, b_int, b_tmp_int);
		mpz_mod(b_int, b_int, ptspace);
		
		// Compute proportional part
		mpz_mul(b_prop, b_kp, b_e);
		mpz_mod(b_prop, b_prop, ptspace);

		// Compute differential part
		mpz_t b_derivative;
		mpz_init(b_derivative);

		mpz_mul(b_derivative, b_kd, b_delta_e);
		mpz_mod(b_derivative, b_derivative, ptspace);
		mpz_mul(b_derivative, b_derivative, b_t_inv);
		mpz_mod(b_derivative, b_derivative, ptspace);
		
		// Add proportional, integral, and derivative parts
		mpz_add(b_u, b_prop, b_int);
		mpz_mod(b_u, b_u, ptspace);
		mpz_add(b_u, b_u, b_derivative);
		mpz_mod(b_u, b_u, ptspace);

		// Recover the control input
		he_decrypt(u_bar, c_u, b_u, p, y, msgsize);

		// Map u_bar to correct interval
		mpz_t test;
		mpz_init(test);
		mpz_sub(test, u_bar, half_ptspace);
		// If test is non-negative, then we got a negative number
		if ( mpz_sgn(test) != -1 )
		{
			mpz_sub(u_bar, u_bar, ptspace);
		}
		mpf_set_z(u, u_bar);

		// Cumulative scaling is 100 000 * 1000 = 100 000 000
		mpf_div_ui(u, u, 100000000);

		current_state = 0.9*current_state + 0.1*(mpf_get_d(u));
		std::cout << "Desired state: " << desirable_state << std::endl;
		std::cout << "Current state: " << current_state << std::endl;

		mpz_set(b_e_prev, b_e);
		mpz_add_ui(e_label, e_label, 1);
		mpz_set(b_t_prev, b_t);
		mpz_add_ui(time_label, time_label, 1);
	}
}
