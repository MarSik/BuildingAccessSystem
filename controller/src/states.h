#ifndef _Door_states_h
#define _Door_states_h

#include <tinyfsm.hpp>
#include <MFRC522Intf.h>
#include <MFRC522Ultralight.h>
#include <Time.h>
#include "countdown.h"
#include "cardmanager.h"
#include "Utils.h"

extern MFRC522IntfSpiOver485 mfrcIntf;
extern MFRC522Ultralight<MFRC522IntfSpiOver485> mfrc522;
extern CardManager<MFRC522IntfSpiOver485> cardManager;

/* temporary prototypes */
void InitReader(bool b_ShowError);
bool ReadCard(uint64_t* uid);
extern bool gb_InitSuccess;

// Events
struct CountdownFinished : public tinyfsm::Event {};
struct UserTyping : public tinyfsm::Event {};
struct BatteryBad : public tinyfsm::Event {};
struct DCFReceived : public tinyfsm::Event {};
struct CardClose : public tinyfsm::Event {};
struct AddCard : public tinyfsm::Event {};
struct ClearCard : public tinyfsm::Event {};
struct BuzzerActivated : public tinyfsm::Event {};
struct BuzzerReleased : public tinyfsm::Event {};

struct AccessSystem : public tinyfsm::Fsm<AccessSystem> {
public:
	/* default reaction for unhandled events */
	void react(tinyfsm::Event const &) {};

    virtual void react(CountdownFinished const &) {}
    virtual void react(UserTyping const &) {}
    virtual void react(BatteryBad const &) {}
    virtual void react(DCFReceived const &) {}

	virtual void entry(void) {};  /* entry actions in some states */
	virtual void exit(void) {};  /* exit actions */
};

class CheckCard;
class ReaderOk;

class ShellInUse : public AccessSystem {
	virtual void entry(void) override {
		countdown(1000);
	};

	virtual void react(UserTyping const &) override {
		countdown(1000);
	};

	virtual void react(CountdownFinished const &) override {
		transit<ReaderOk>();
	};
};

class ReaderOk : public AccessSystem {
	virtual void entry(void) override {
		countdown(500);
	}

	virtual void react(UserTyping const &) override {
		transit<ShellInUse>();
	};

	virtual void react(CountdownFinished const &) override {
		transit<CheckCard>();
	};
};

class CheckCard;
class ReaderErrorWait;
class ReaderInitialize;

class ReaderError : public AccessSystem {
	virtual void entry(void) override {
		// TODO turn on error LED
		countdown(1000);
	};

	virtual void react(CountdownFinished const &) override {
		// TODO turn off error LED
		transit<ReaderErrorWait>();
	};
};

class ReaderErrorWait : public AccessSystem {
	virtual void entry(void) override {
		countdown(2000);
	};

	virtual void react(CountdownFinished const &) override {
		transit<ReaderInitialize>();
	}
};

class AccessDenied : public AccessSystem {
	virtual void entry(void) override {
		// TODO turn on error LED
		countdown(1000);
	};

	virtual void react(CountdownFinished const &) override {
		// TODO turn off error LED
		transit<ReaderOk>();
	};
};

class AccessAllowed : public AccessSystem {
	virtual void entry(void) override {
		// TODO turn on OK LED + door
		countdown(1000);
	};

	virtual void react(CountdownFinished const &) override {
		// TODO turn off OK LED + door
		transit<ReaderOk>();
	};
};

class CheckCard : public AccessSystem {
public:
	virtual void entry(void) override {
		uint64_t uid;
		bool oneGoodCard = false;
		bool oneCard = false;

		// There might be more than one card in the field
		// Halt all cards that do not match and try again
		while (!oneGoodCard && ReadCard(&uid)) {
			oneCard = true;

			if (!mfrc522.UltralightC_Authenticate(APPLICATION_KEY)) // e.g. Error while authenticating with master key
			{
				mfrc522.PICC_HaltA();
				Utils::Print("Card authentication failed!", LF);
				continue;
			} else {
				Utils::Print("Card successfully authenticated!", LF);
			}

			const CardInfo &signature = cardManager.read_signature();
			if (!signature.valid) {
				mfrc522.PICC_HaltA();
				Utils::Print("Card not configured for this application.", LF);
				continue;
			}

			if (signature.special_app & CardInfo::ACCESS_BT) {
				// TODO Enable bluetooth pairing until timeout
			}

			AccessRule rule;
			if (!cardManager.get_rule(DOOR_ID, rule)) {
				mfrc522.PICC_HaltA();
				Utils::Print("Door rule not accessible.", LF);
				continue;
			}

			if (!rule.check_crc()) {
				mfrc522.PICC_HaltA();
				Utils::Print("Door rule corrupted.", LF);
				continue;
			}

			const time_t _now = now();

			if (!rule.check_access(weekday(_now), hour(_now))) {
				mfrc522.PICC_HaltA();
				Utils::Print("Card is not allowed to access this door.", LF);
				continue;
			}

			oneGoodCard = true;
		}

		if (!gb_InitSuccess) {
			transit<ReaderError>();
		} else if (oneGoodCard) {
			transit<AccessAllowed>();
		} else if (oneCard) {
			transit<AccessDenied>();
		} else {
			transit<ReaderOk>();
		}
	}

	virtual void exit() override {
		mfrc522.PCD_AntennaOff();
	}
};

class ReaderInitialize : public AccessSystem {
	virtual void entry(void) override {
		// Initialize and move to proper state
		InitReader(false);
		if (gb_InitSuccess) {
			transit<ReaderOk>();
		} else {
			transit<ReaderError>();
		}
	}
};

#endif
