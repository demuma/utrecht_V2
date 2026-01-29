remote:
	@docker compose build remote && docker compose up remote

robot:
	@python3 detect_devices.py
	@docker compose build robot && docker compose up robot

.PHONY: remote robot
