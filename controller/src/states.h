#ifndef _Door_states_h
#define _Door_states_h

#include <tinyfsm.hpp>
#include <MFRC522Intf.h>
#include <MFRC522Ultralight.h>
#include <Time.h>
#include "countdown.h"
#include "cardmanager.h"
#include "Utils.h"
#include "config.h"
#include "hal.h"

extern MFRC522IntfSpiOver485 mfrcIntf;
extern MFRC522Ultralight<MFRC522IntfSpiOver485> mfrc522;
extern CardManager<MFRC522IntfSpiOver485> cardManager;

/* temporary prototypes */
void InitReader(bool b_ShowError);
bool ReadCard(uint64_t* uid);
extern bool gb_InitSuccess;
bool ReadKeyboardInput();

// Events
struct CountdownFinished : public tinyfsm::Event {};
struct UserTyping : public tinyfsm::Event {};
struct BatteryBad : public tinyfsm::Event {};
struct DCFReceived : public tinyfsm::Event {};
struct AddCard : public tinyfsm::Event {};
struct TestCard : public tinyfsm::Event {};
struct ClearCard : public tinyfsm::Event {};
struct BuzzerActivated : public tinyfsm::Event {};
struct BuzzerReleased : public tinyfsm::Event {};
struct PasswordEntered : public tinyfsm::Event {};
struct BadPasswordEntered : public tinyfsm::Event {};
struct ClearAppKeyRequest : public tinyfsm::Event {};

struct AccessSystem : public tinyfsm::Fsm<AccessSystem> {
public:
	/* default reaction for unhandled events */
	void react(tinyfsm::Event const &) {};

    virtual void react(CountdownFinished const &) {}
    virtual void react(UserTyping const &) {}
    virtual void react(BatteryBad const &) {}
    virtual void react(DCFReceived const &) {}
    virtual void react(BadPasswordEntered const &) {}
    virtual void react(PasswordEntered const &) {}
    virtual void react(AddCard const &) {}
    virtual void react(ClearCard const &) {}
    virtual void react(TestCard const &) {}
    virtual void react(ClearAppKeyRequest const &) {}

	virtual void entry(void) {};  /* entry actions in some states */
	virtual void exit(void) {};  /* exit actions */
};

class CheckCard;
class ReaderOk;
class ShellInUse;
class ReaderError;

class WaitForYesNo : public AccessSystem {
    virtual void yes() = 0;
    virtual void no() = 0;
    virtual void nothing() = 0;

    virtual void entry(void) override {
        Serial.write("Please confirm the action. Press Y if you really want it. N otherwise.\r\n"
                     "Default is No. You have 30 seconds.\r\n");
        countdown(30000);
    };

    virtual void react(UserTyping const &) override {
        const int ch = Serial.read();
        switch (ch) {
            case 'n':
            case 'N':
            case '\n':
            case 27: // ESC
                no();
                break;
            case 'y':
            case 'Y':
            case 'a':
            case 'A':
                yes();
                break;
            default:
                break;
        }
    };

    virtual void react(CountdownFinished const &) override {
        nothing();
    }
};

class ReallyClearAppKey: public WaitForYesNo {
    virtual void yes() {
        clearApplicationKey();
        transit<ShellInUse>();
    }

    virtual void no() {
        Serial.print("Aborting.\r\n");
        transit<ShellInUse>();
    }

    virtual void nothing() {
        Serial.print("Timeout while waiting for answer. Aborting.\r\n");
        transit<ShellInUse>();
    }
};

class WaitForCard : public AccessSystem {
    static const uint16_t MAX_ATTEMPTS = 300; // 30 sec total
    static uint16_t attempts;

    virtual void noCardFound() = 0;
    virtual void cardFound(uint64_t &uid) = 0;

    virtual void entry(void) override {
        attempts = 0;
        Utils::Print("Please approximate the card to the reader now!\r\nYou have 30 seconds. Abort with ESC.\r\n");
        countdown(100);
    };

    virtual void react(UserTyping const &) override {
        if (Serial.read() == 27) { // ESC
            Utils::Print("Aborted.\r\n");
            noCardFound();
        }
    };

    virtual void react(CountdownFinished const &) override {
        uint64_t uid;
        if (ReadCard(&uid)) {
            Utils::Print("Card found.\r\n");
            cardFound(uid);
        } else if (!gb_InitSuccess) {
            Utils::Print("Reader connection failed.\r\n");
            transit<ReaderError>();
        } else if (++attempts > MAX_ATTEMPTS) {
            Utils::Print("Timeout waiting for card.\r\n");
            noCardFound();
        } else {
            SetLED(attempts % 4 ? LED_OFF : LED_GREEN);
            countdown(200);
        }
    };

    void exit() override {
        SetLED(LED_OFF);
        mfrc522.PCD_AntennaOff();
    }
};

class ShellTestCard : public WaitForCard {
    void noCardFound() override {
        transit<ShellInUse>();
    }

    void cardFound(uint64_t &uid) override {
        if (cardManager.authenticate()) {
            Utils::Print("Authentication to card succeeded", LF);
        } else {
            Utils::Print("Authentication to card failed", LF);
        }
        transit<ShellInUse>();
    }
};

class RestoreCard : public WaitForCard {
    void noCardFound() override {
        transit<ShellInUse>();
    }

    void cardFound(uint64_t &uid) override {
        // TODO compute per-card key from app key and uid
        if (cardManager.authenticate()) {
            mfrc522.PCD_AntennaOff();
            Utils::Print("Authentication to card succeeded", LF);
        } else {
            Utils::Print("Authentication to card failed", LF);
        }

        if (cardManager.reset_card()) Utils::Print("Restore success\r\n");
        else                          Utils::Print("Restore failed\r\n");

        transit<ShellInUse>();
    }
};

class PersonalizeCard : public WaitForCard {
    void noCardFound() override {
        transit<ShellInUse>();
    }

    void cardFound(uint64_t &uid) override {
        if (cardManager.personalize_card()) {
            mfrc522.PCD_AntennaOff();
            Utils::Print("Card configured for access", LF);
        } else {
            Utils::Print("Card configuration failed", LF);
        }
        transit<ShellInUse>();
    }
};

class BadPasswordDelay : public AccessSystem {
    virtual void entry(void) override {
        SetLED(LED_RED);
        countdown(1000);
    };

    virtual void react(CountdownFinished const &) override {
        SetLED(LED_OFF);
        transit<ShellInUse>();
    };
};

class ShellInUse : public AccessSystem {
	virtual void entry(void) override {
		countdown(1000);
	};

	virtual void react(UserTyping const &) override {
		countdown(1000);
		ReadKeyboardInput();
	};

	virtual void react(CountdownFinished const &) override {
		transit<ReaderOk>();
	};

    virtual void react(BadPasswordEntered const &) override {
        transit<BadPasswordDelay>();
    };

    virtual void react(ClearCard const &) override {
        transit<RestoreCard>();
    };

    virtual void react(AddCard const &) override {
        transit<PersonalizeCard>();
    };

    virtual void react(TestCard const &) override {
        transit<ShellTestCard>();
    };

    virtual void react(ClearAppKeyRequest const &) override {
        transit<ReallyClearAppKey>();
    }

    virtual void exit() {
    }
};

class ReaderOk : public AccessSystem {
	virtual void entry(void) override {
		countdown(250);
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
        SetLED(LED_RED);
		countdown(1000);
	};

	virtual void react(CountdownFinished const &) override {
        SetLED(LED_OFF);
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
        SetLED(LED_RED);
		countdown(1000);
	};

	virtual void react(CountdownFinished const &) override {
        SetLED(LED_OFF);
		transit<ReaderOk>();
	};
};

class AccessAllowed : public AccessSystem {
	virtual void entry(void) override {
        Utils::Print("Opening door!\r\n");
        SetLED(LED_GREEN);
        Utils::WritePin(DOOR_PIN, HIGH);
		countdown(1000);
	};

	virtual void react(CountdownFinished const &) override {
        transit<ReaderOk>();
	};

    virtual void exit() {
        Utils::Print("Closing door!\r\n");
        Utils::WritePin(DOOR_PIN, LOW);
        SetLED(LED_OFF);
    }
};

class CheckCard : public AccessSystem {
public:
	virtual void entry(void) override {
		uint64_t uid = 0;
        uint64_t lastUid = 0;
		bool cardGood = false;
		bool cardPresent = false;

		// There might be more than one card in the field
		// Halt all cards that do not match and try again
        // Also stop checking if the same card is read twice
		while (ReadCard(&uid) && uid != lastUid) {
			cardPresent = true;

            if (!cardManager.authenticate()) // e.g. Error while authenticating with master key
			{
				mfrc522.PICC_HaltA();
				Utils::Print("Card authentication failed!", LF);
				continue;
			} else {
				Utils::Print("Card successfully authenticated!", LF);
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

            lastUid = uid;
            cardGood = true;
            break;
		}

		if (!gb_InitSuccess) {
			transit<ReaderError>();
		} else if (cardGood) {
			transit<AccessAllowed>();
		} else if (cardPresent) {
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
        // A longer pause is required to assure that the condensator at VOLTAGE_MEASURE_PIN
        // has been charged before the battery voltage is measured for the first time.
        SetLED(LED_GREEN);
        Utils::WritePin(DOOR_PIN, LOW);
        countdown(1000);
    }

    virtual void react(CountdownFinished const &) override {
		// Initialize and move to proper state
		InitReader(false);
		if (gb_InitSuccess) {
			transit<ReaderOk>();
		} else {
			transit<ReaderError>();
		}
	}

    virtual void exit() {
        SetLED(LED_OFF);
    }
};

#endif
