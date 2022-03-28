#include "LabeledHE/labeled_he.h"
#include "labeled_he_control.h"

#include <gmp.h>
#include <iostream>
#include <unistd.h>

int main()
{
	std::cout << "Hello World\n";

	// Control parameters - Proportional gain
	mpz_t kp;
	mpz_init_set_ui(kp, 1); // Not scaled

	// Cryptographic parameters
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

	mpz_t kp_label, e_label;
	mpz_init_set_ui(kp_label, 5);
	mpz_init_set_ui(e_label, 2);
	
	mpz_t b_kp, b_e;
	mpz_init(b_kp);
	mpz_init(b_e);

	he_ct c_kp, c_e;

	mpz_t c_u;
	mpz_init(c_u);

	he_encrypt(&c_kp, state, &hc_cs, b_kp, kp, y, N, kp_label, msgsize);

	// Initialize proportional control
	Labeled_he_p_control he_p_controller(&c_kp, y, N, msgsize);

	mpf_t u;
	mpf_init(u);

	// ASSUME STATE BETWEEN -1 and 1
	double desirable_state = 10;
	double current_state = 0;
	double error;
	mpz_t error_bar;
	mpz_init(error_bar);
	for (int i = 0; i < 1000; i++)
	{
		// Switch sign of desired state every 250 samples
                if ( (i != 0) && (i % 250 == 0) )
                {
			std::cout << "\n\nFLIPPING DESIRABLE STATE\n\n";
                        desirable_state = -desirable_state;
                }

                // Sleep for 0.1 second
                usleep(100000);

		error = desirable_state - current_state;

		// Map error to non-negative integer!
		error = error * 100000; // Scaled by 100 000
		mpz_set_d(error_bar, error);
		mpz_mod(error_bar, error_bar, ptspace);
		
		// Encrypt state error
		he_encrypt(&c_e, state, &hc_cs, b_e, error_bar, y, N, e_label, msgsize);
		
		// Iteratore controller
		he_p_controller.iterate(c_u, &c_e);

		mpz_t u_bar, b_u, b_tmp;
		mpz_init(u_bar);;
		mpz_init(b_u);
		mpz_init(b_tmp);

		// Decrypt
		mpz_mul(b_u, b_kp, b_e);
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

		// Cumulative scaling is 100 000
		mpf_div_ui(u, u, 100000);

		current_state = 0.9*current_state + mpf_get_d(u);
		std::cout << "Desirable state: " << desirable_state << std::endl;
		std::cout << "Current state: " << current_state << std::endl;
	}
}
