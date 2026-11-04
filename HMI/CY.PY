def _run_pieces(
        self, length_mm: float, qty: int, rpm: float, start_rep: int
    ) -> bool:
        """Bucle de piezas desde start_rep. True = excepción interna ya en
        Pause y el operador hizo Resume → reentrar desde la pieza en curso.
        Una excepción no cierra el lote (solo Stop / Abortar / fin de lote)."""
        try:
            # Inicia rutina
            target_mm = 0.0
            completed = start_rep - 1
            handoff_ready = False
            for rep in range(start_rep, qty + 1):
                if self._pf_trigger_prearmed_rep == int(rep):
                    self._pf_trigger_sent_this_piece = True
                    self._pf_trigger_prearmed_rep = None
                else:
                    self._pf_trigger_sent_this_piece = False
                while True:
                    early_exit = True
                    materialist_only = False
                    if self._lot_purge_request is not None:
                        self._run_lot_purge()
                        if self._should_abort():
                            break
                        handoff_ready = False
                        self._recovery_skip_feed = True
                        continue
                    for _piece_attempt in (0,):
                        if self._gate("listo" if rep == 1 else "rep-start"):
                            break
                        if self._e050_materialist_requested and not self._e050_finish_piece:
                            if not self._run_e050_materialist_recovery():
                                early_exit = True
                                break
                            materialist_only = True
                            early_exit = True
                            break
                        finish_cut_only = bool(
                            self._recovery_skip_feed and self._pending_cut_resume
                        )
                        recovery_restart = self._recovery_skip_feed
                        if not finish_cut_only:
                            self._set_progress(rep, 0, qty)
                            self._last_lineal_sec = 0.0
                            self._last_lineal_mm = 0.0
                            analysis_begin_piece(int(rep), int(qty))
                            self._mark_piece_clock_start()
                            self._cutter_set_mono = None
                            self._cutter_res_mono = None
                        else:
                            self._host.cycle_log(
                                "Recovery: completar corte (sin feed/lineal)"
                            )
                            target_mm = self._live_cut_target_mm()
                            self._recovery_skip_feed = False
                        # Activa Holder y Encoder al iniciar o recuperar
                        if not finish_cut_only and (rep == 1 or recovery_restart):
                            if self._enter(rep, qty, "holder_on"):
                                break
                            hold_op = self._begin_op("holder_on")
                            arm_ok = self._arm_holder_encoder()
                            self._end_op(hold_op, ok=arm_ok)
                            if not arm_ok:
                                self._host.cycle_log(
                                    "Holder+Encoder ON: envío al PLC falló"
                                )
                                self._raise_fault("E066")
                                break
                            self._host.cycle_log(
                                f"Holder+Encoder ON (inicio pieza {rep})"
                            )
                            if self._after_step("holder_on"):
                                break
                            if self._cycle_delay(rep, qty, "wait_holder_on"):
                                break
                        # Verifica HOME antes de alimentar tras recovery
                        if (
                            not finish_cut_only
                            and recovery_restart
                            and not self._ensure_asda_at_zero(reason="recovery")
                        ):
                            break
                        if finish_cut_only:
                            if self._enter(rep, qty, "grippers_on"):
                                break
                            grip_op = self._begin_op("grippers_on")
                            grip_ok = self._host.cmd_plc_gripper(True)
                            self._end_op(grip_op, ok=grip_ok)
                            if not grip_ok:
                                self._host.cycle_log("Pinzas ON: envío al PLC falló")
                                self._raise_fault("E066")
                                break
                            self._host.cycle_log("Pinzas ON (recovery corte)")
                            if self._after_step("grippers_on"):
                                break
                            if self._cycle_delay(rep, qty, "wait_grippers_on"):
                                break
                        if not finish_cut_only:
                            # Prepara PreFeeder
                            if self._enter(rep, qty, "pf_trigger"):
                                break
                            will_run_feed = False
                            if handoff_ready:
                                pass
                            elif self._material_feed_done and (
                                self._recovery_skip_feed or self._recovery_after_error
                            ):
                                pass
                            elif (
                                (
                                    rep == 1
                                    or self._recovery_skip_feed
                                    or self._recovery_after_error
                                )
                                and self._feed_reference_visible()
                            ):
                                pass
                            else:
                                will_run_feed = True
                            if self._pf_trigger_sent_this_piece:
                                self._host.cycle_log(
                                    "trigger PreFeeder: omitido "
                                    "(ya mandado pre-feed / handoff)"
                                )
                            elif will_run_feed:
                                if not self._do_pf_trigger(rep):
                                    break
                            else:
                                self._host.cycle_log(
                                    "trigger PreFeeder: omitido "
                                    "(sin alimentación Motion en este paso)"
                                )
                            if self._after_step("pf_trigger"):
                                break
                            # Mueve feeder
                            if self._enter(rep, qty, "feed"):
                                break
                            if handoff_ready:
                                handoff_ready = False
                                self._recovery_skip_feed = False
                                self._material_feed_done = True
                                self._host.cycle_log(
                                    "Feed: handoff (ya alimentado post-HOME, ASDA=0)"
                                )
                            elif self._material_feed_done and (
                                self._recovery_skip_feed or self._recovery_after_error
                            ):
                                self._recovery_skip_feed = False
                                sides_txt = "".join(self._feed_side_list())
                                self._host.cycle_log(
                                    f"Feed omitido (ya alimentado+offset) lados={sides_txt}"
                                )
                            elif (
                                (
                                    rep == 1
                                    or self._recovery_skip_feed
                                    or self._recovery_after_error
                                )
                                and self._feed_reference_visible()
                            ):
                                c2_or_rec = (
                                    self._recovery_skip_feed or self._recovery_after_error
                                )
                                self._recovery_skip_feed = False
                                self._material_feed_done = True
                                sides_txt = "".join(self._feed_side_list())
                                if c2_or_rec:
                                    self._host.cycle_log(
                                        f"Feed omitido (recovery/recovery): láser ya ON "
                                        f"lados={sides_txt}"
                                    )
                                else:
                                    self._host.cycle_log(
                                        f"Feed omitido (referencia láser visible) "
                                        f"lados={sides_txt}"
                                    )
                            else:
                                self._recovery_skip_feed = False
                                if not self._run_feed():
                                    break
                            if self._after_step("feed"):
                                break
                            # Offset
                            if self._enter(rep, qty, "offset"):
                                break
                            if self._after_step("offset"):
                                break
                            # Cierra pinzas
                            if self._enter(rep, qty, "grippers_on"):
                                break
                            grip_op = self._begin_op("grippers_on")
                            grip_ok = self._host.cmd_plc_gripper(True)
                            self._end_op(grip_op, ok=grip_ok)
                            if not grip_ok:
                                self._host.cycle_log("Pinzas ON: envío al PLC falló")
                                self._raise_fault("E066")
                                break
                            self._host.cycle_log("Pinzas ON (cierran)")
                            if self._after_step("grippers_on"):
                                break
                            if self._cycle_delay(rep, qty, "wait_grippers_on"):
                                break
                            if self._enter(rep, qty, "enc_set0"):
                                break
                            self._host.cycle_log(
                                "enc_set0: omitido (lineal TCP no usa OM)"
                            )
                            if self._after_step("enc_set0"):
                                break
                            # Abre Holder y Encoder para avanzar el material
                            if self._enter(rep, qty, "holder_off"):
                                break
                            hold_off_op = self._begin_op("holder_off")
                            open_ok = self._open_holder_encoder()
                            self._end_op(hold_off_op, ok=open_ok)
                            if not open_ok:
                                self._host.cycle_log(
                                    "Holder+Encoder OFF: envío al PLC falló"
                                )
                                self._raise_fault("E066")
                                break
                            self._host.cycle_log(
                                "Holder+Encoder OFF (abren para lineal)"
                            )
                            if self._after_step("holder_off"):
                                break
                            if self._cycle_delay(rep, qty, "wait_holder_open"):
                                break

                            if self._enter(rep, qty, "lineal_fwd"):
                                break
                            # Avanza material a la posición de corte
                            target_mm = self._live_cut_target_mm()
                            abs_target_mm = abs(float(target_mm))
                            cut_off = abs_target_mm - abs(
                                float(self._lot_length_mm)
                            )
                            sides = CycleConfig.normalize_feed_sides(
                                self.get_config().feed_sides
                            )
                            self._host.cycle_log(
                                f"Lineal MOVE TCP modelMm={length_mm:.1f} "
                                f"cutOffset={cut_off:.1f}→targetAbsMm={abs_target_mm:.1f} "
                                f"sides={sides} rpm={self._lot_rpm:g}"
                            )
                            if not self._run_lineal_abs(abs_target_mm):
                                break
                            if self._after_step("lineal_fwd"):
                                break
                            if self._cycle_delay(rep, qty, "wait_linear_done"):
                                break
                            self._pending_cut_resume = True
                        # Cierra Holder y Encoder antes del corte
                        if self._enter(rep, qty, "holder_precut"):
                            break
                        precut_op = self._begin_op("holder_precut")
                        arm_ok = self._arm_holder_encoder()
                        self._end_op(precut_op, ok=arm_ok)
                        if not arm_ok:
                            self._host.cycle_log(
                                "Holder+Encoder ON pre-corte: envío al PLC falló"
                            )
                            self._raise_fault("E066")
                            break
                        self._host.cycle_log("Holder+Encoder ON (cierran pre-corte)")
                        if self._after_step("holder_precut"):
                            break
                        if self._cycle_delay(rep, qty, "wait_holder_precut"):
                            break
                        # Corta la pieza
                        cut_sides = CycleConfig.normalize_feed_sides(self.get_config().feed_sides)
                        if self._enter(rep, qty, "cutter_on"):
                            break
                        if (
                            self._use_prefeeder()
                            and self._host.pf_has_fault()
                        ):
                            self._raise_current_pf_fault("Corte: PreFeeder en error")
                            if (
                                self._recovery_after_error
                                or self._e050_finish_piece
                            ):
                                self._host.cycle_log(
                                    "recovery: completar corte pese a EXXX PF"
                                )
                            elif self._should_abort():
                                self._host.cycle_log("Corte abortado: PreFeeder Error")
                                break
                            elif self._pause.is_set():
                                if self._wait_paused_for_resume(
                                    "recovery: Pause por EXXX PF antes del corte"
                                ):
                                    break
                                break
                            else:
                                self._host.cycle_log("Corte abortado: PreFeeder Error")
                                break
                        cut_op = self._begin_op("corte")
                        self._host.cycle_log(f"Cortador ON (Set) lados={cut_sides}")
                        if not self._host.cmd_plc_cutters(
                            True, sides=cut_sides, force=True
                        ):
                            self._end_op(cut_op, ok=False)
                            self._host.cycle_log("Cortador ON: envío al PLC falló")
                            self._raise_fault("E066")
                            break
                        self._mark_cutter_set()
                        if self._after_step("cutter_on"):
                            self._end_op(cut_op, ok=False)
                            self._host.cmd_plc_cutters(False, sides=cut_sides, force=True)
                            break
                        if self._cycle_delay(rep, qty, "wait_cutter_pulse"):
                            self._end_op(cut_op, ok=False)
                            self._host.cmd_plc_cutters(False, sides=cut_sides, force=True)
                            break
                        if self._enter(rep, qty, "cutter_off"):
                            self._end_op(cut_op, ok=False)
                            self._host.cmd_plc_cutters(False, sides=cut_sides, force=True)
                            break
                        if not self._host.cmd_plc_cutters(
                            False, sides=cut_sides, force=True
                        ):
                            self._end_op(cut_op, ok=False)
                            self._host.cycle_log("Cortador OFF: envío al PLC falló")
                            self._raise_fault("E066")
                            break
                        self._mark_cutter_res()
                        cut_sec = self._end_op(cut_op, ok=True)
                        self._host.cycle_log(
                            f"Cortador OFF (Res) lados={cut_sides}"
                            f" · {self._fmt_op(cut_sec)}"
                        )
                        if self._after_step("cutter_off"):
                            break
                        if self._cycle_delay(rep, qty, "wait_cutter_post"):
                            break
                        if self._enter(rep, qty, "cutter_settle"):
                            break
                        if self._ensure_cutter_settled_before_travel():
                            break
                        if not self._arm_tfeed_for_next_piece_after_cut(rep, qty):
                            break
                        # Deposita la pieza
                        wip_pos_signed: float | None = None
                        wip_start_mm: float | None = None
                        if self._enter(rep, qty, "deposit"):
                            break
                        extra = self._deposit_extra_mm(rep)
                        if abs(extra) > 0.01:
                            deposit_target = target_mm + extra
                            batch_n = self._deposit_batch_index(rep)
                            self._host.clear_motion_wait_flags()
                            self._host.cycle_log(
                                f"Depósito MOVE → {deposit_target:.1f} mm "
                                f"(batch {batch_n}, extra={extra:.1f})"
                            )
                            dep_cmd = self._begin_op("depósito_cmd")
                            dep_sent = self._host.cmd_motion_move_mm(deposit_target, rpm)
                            dep_cmd_sec = self._end_op(dep_cmd, ok=dep_sent)
                            if not dep_sent:
                                if not self._should_abort() and not self._fault:
                                    kind = ""
                                    if hasattr(self._host, "last_move_fail_kind"):
                                        kind = self._host.last_move_fail_kind()
                                    if kind == "transport":
                                        self._raise_fault("E065")
                                    else:
                                        self._raise_fault("deposit_cmd")
                                break
                            dep_op = self._begin_op("depósito")
                            if not self._wait_motion(
                                target_mm=deposit_target,
                                reissue=lambda: self._host.cmd_motion_move_mm(
                                    deposit_target, rpm
                                ),
                            ):
                                self._end_op(dep_op, ok=False)
                                break
                            wip_pos_signed = float(deposit_target)
                            wip_start_mm = abs(wip_pos_signed)
                            dep_sec = self._end_op(dep_op, ok=True)
                            self._host.cycle_log(
                                f"Depósito MOVE ok → {deposit_target:.1f} mm "
                                f"(WIP start ref {wip_start_mm:.1f})"
                                f" · move={self._fmt_op(dep_sec)} "
                                f"cmd={self._fmt_op(dep_cmd_sec)}"
                            )
                        else:
                            wip_pos_signed = float(target_mm)
                            wip_start_mm = abs(wip_pos_signed)
                            self._host.cycle_log(
                                f"WIP start ref=lineal_target {wip_start_mm:.1f} mm "
                                f"(sin depósito extra)"
                            )
                        if not self._arm_prefetch_feed_after_deposit_reached(rep, qty):
                            break
                        if self._after_step("deposit"):
                            break
                        if self._cycle_delay(rep, qty, "wait_deposit_dwell"):
                            break
                        # Abre pinzas
                        if self._enter(rep, qty, "grippers_off"):
                            break
                        grip_off_op = self._begin_op("grippers_off")
                        grip_ok = self._host.cmd_plc_gripper(False)
                        self._end_op(grip_off_op, ok=grip_ok)
                        if not grip_ok:
                            self._host.cycle_log("Pinzas OFF: envío al PLC falló")
                            self._raise_fault("E066")
                            break
                        self._host.cycle_log("Pinzas OFF (abren)")
                        if self._after_step("grippers_off"):
                            break
                        if self._cycle_delay(rep, qty, "wait_gripper_release"):
                            break
                        # Despeja la pieza de las pinzas
                        if self._enter(rep, qty, "gripper_clearance"):
                            break
                        clr = abs(float(self.get_config().gripper_clearance_mm))
                        if clr > 0.01 and wip_pos_signed is not None:
                            clearance_target = self._clearance_target_mm(
                                wip_pos_signed, float(target_mm), clr
                            )
                            self._arm_motion_leg(
                                prefetch_running=self._prefetch_motion_parallel()
                            )
                            self._host.cycle_log(
                                f"Despeje MOVE → {clearance_target:.1f} mm "
                                f"(+|clearance|={clr:.1f})"
                            )
                            clr_cmd = self._begin_op("despeje_cmd")
                            clr_sent = self._host.cmd_motion_move_mm(clearance_target, rpm)
                            clr_cmd_sec = self._end_op(clr_cmd, ok=clr_sent)
                            if not clr_sent:
                                if not self._should_abort() and not self._fault:
                                    kind = ""
                                    if hasattr(self._host, "last_move_fail_kind"):
                                        kind = self._host.last_move_fail_kind()
                                    if kind == "transport":
                                        self._raise_fault("E065")
                                    else:
                                        self._raise_fault("clearance_cmd")
                                break
                            clr_op = self._begin_op("despeje")
                            if not self._wait_motion(
                                target_mm=clearance_target,
                                reissue=lambda: self._host.cmd_motion_move_mm(
                                    clearance_target, rpm
                                ),
                            ):
                                self._end_op(clr_op, ok=False)
                                break
                            wip_pos_signed = float(clearance_target)
                            wip_start_mm = abs(wip_pos_signed)
                            clr_sec = self._end_op(clr_op, ok=True)
                            self._host.cycle_log(
                                f"Despeje MOVE ok → {wip_pos_signed:.1f} mm; "
                                f"WIP fin ref={wip_start_mm:.1f} mm"
                                f" · move={self._fmt_op(clr_sec)} "
                                f"cmd={self._fmt_op(clr_cmd_sec)}"
                            )
                        else:
                            self._host.cycle_log(
                                "Despeje post-pinzas: omitido "
                                f"(clearance={clr:.1f} mm)"
                            )
                        if self._after_step("gripper_clearance"):
                            break
                        # Regresa a HOME
                        if self._enter(rep, qty, "home"):
                            break
                        if not self._home_with_wip_delivery(
                            wip_start_mm,
                            self._lot_rpm,
                            prefetch_running=self._prefetch_motion_parallel(),
                        ):
                            break
                        if self._after_step("home"):
                            break
                        # Cierra prefetch (barrera HOME) o feed post-HOME si no hubo prefetch.
                        if self._enter(rep, qty, "feed_after_home"):
                            break
                        if not self._complete_prefetch_after_home(rep, qty):
                            if self._restart_piece or self._fault:
                                break
                            if self._recovery_after_error:
                                self._host.cycle_log(
                                    "Feed post-HOME falló — recovery sigue a "
                                    "post_piece (Pause; Reset→Resume)"
                                )
                            else:
                                self._raise_fault("feed_incomplete")
                                break
                        elif int(rep) < int(qty):
                            handoff_ready = True
                        if self._after_step("feed_after_home"):
                            break
                        # Asienta y cierra el ciclo de pieza
                        if self._cycle_delay(rep, qty, "wait_asentar"):
                            break
                        if self._enter(rep, qty, "post_piece"):
                            break
                        piece_sec = self._freeze_piece_clock()
                        if self._after_step("post_piece"):
                            break
                        completed = rep
                        with self._lock:
                            self._pieces_done = rep
                            if qty > 0:
                                self._progress = max(
                                    self._progress,
                                    int(min(100, round(100.0 * rep / qty))),
                                )
                        self._log_piece_ok(rep, qty, piece_sec)
                        self._host.increment_maintenance_cycle_count()
                        self._host.cycle_notify()
                        recovery_ok, invalidate_handoff = self._after_piece_recovery(
                            rep, qty
                        )
                        if not recovery_ok:
                            early_exit = True
                            break
                        if invalidate_handoff:
                            handoff_ready = False
                            self._reset_prefetch()
                        early_exit = False
                    if materialist_only:
                        continue
                    if not early_exit:
                        break  # pieza OK → siguiente rep
                    if self._consume_restart_piece():
                        handoff_ready = False
                        self._reset_prefetch()
                        self._recovery_skip_feed = True
                        self._host.cycle_log(
                            f"recovery: reinicio pieza {rep}/{qty} desde step 0"
                        )
                        continue
                    if (
                        (self._recovery_after_error or self._fault)
                        and not self._should_abort()
                        and self._wait_recovery_resume_hold()
                    ):
                        handoff_ready = False
                        self._recovery_skip_feed = True
                        self._recovery_skip_pf_trigger = bool(
                            self._pf_trigger_sent_this_piece
                        )