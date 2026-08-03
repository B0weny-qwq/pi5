# TASK5

TASK5 uses the exact TASK4 vision, PDI, motor control, encoder-acceleration
feedforward maps, video stream and runtime logging. Its only behavioral change
is a longer evaluation window.

- TASK4 evaluation: `8.0 s`
- TASK5 evaluation: `20.0 s`
- TASK5 log prefix: `task5_balance_*`
- TASK5 executable: `./ball2_task5_velocity`

Build on the Raspberry Pi:

```bash
cd /home/boweny/111.5_p
chmod +x build.sh
./build.sh
```

Run:

```bash
./ball2_task5_velocity
```

Operation remains identical: place the ball at O, wait for
`READY - PRESS SPACE`, then press Space to start or abort.

To change only the TASK5 duration, edit `BALL_TASK_EVALUATION_SECONDS` in
`111.5_p/main.cpp`. All control parameters remain owned by `111.4_p/main.cpp`.
