import React, { useState } from 'react';
import { ShieldOff, X } from 'lucide-react';
import { useApp } from '../context/AppContext';
import { prefeederAction } from '../api/hmiApi';
import {
  hmiAssistHint,
  hmiBtnPrimary,
  hmiBtnSecondary,
  hmiFieldInput,
  hmiFieldLabel,
  hmiIconBtn,
  hmiPanelCls,
} from '../styles/hmiUi';

interface PfHolguraOmitModalProps {
  isOpen: boolean;
  side: 'L' | 'R';
  active: boolean;
  onClose: () => void;
}

export const PfHolguraOmitModal: React.FC<PfHolguraOmitModalProps> = ({
  isOpen,
  side,
  active,
  onClose,
}) => {
  const { t } = useApp();
  const [employeeId, setEmployeeId] = useState('');
  const [error, setError] = useState<'invalid_employee' | 'server' | null>(null);
  const [submitting, setSubmitting] = useState(false);

  if (!isOpen) return null;

  const handleConfirm = async () => {
    if (employeeId.length !== 5) {
      setError('invalid_employee');
      return;
    }
    setSubmitting(true);
    setError(null);
    try {
      const result = await prefeederAction('toggle_holgura_omit', {
        side,
        employeeId,
      });
      if (result.ok) {
        setEmployeeId('');
        onClose();
        return;
      }
      const err = String(result.error || '');
      if (err.includes('empleado') || err === 'invalid_employee') {
        setError('invalid_employee');
      } else {
        setError('server');
      }
    } catch {
      setError('server');
    } finally {
      setSubmitting(false);
    }
  };

  const handleClose = () => {
    setEmployeeId('');
    setError(null);
    onClose();
  };

  const sideLabel = side === 'L' ? t('side_l') : t('side_r');

  return (
    <div className="fixed inset-0 z-[60] flex items-center justify-center p-4 animate-fade-in">
      <div
        className="fixed inset-0 bg-slate-900/50 backdrop-blur-xs"
        onClick={handleClose}
      />
      <div
        role="dialog"
        aria-modal="true"
        aria-labelledby="pf-holgura-omit-title"
        className={`relative z-10 w-full max-w-md p-4 shadow-2xl sm:p-5 ${hmiPanelCls}`}
      >
        <div className="mb-4 flex items-start justify-between gap-3">
          <div className="flex items-center gap-2">
            <ShieldOff className="h-6 w-6 text-amber-600 dark:text-amber-400" />
            <h2
              id="pf-holgura-omit-title"
              className="text-lg font-bold text-slate-900 dark:text-slate-100"
            >
              {active ? t('pf_holgura_omit_off_title') : t('pf_holgura_omit_on_title')} ({sideLabel})
            </h2>
          </div>
          <button type="button" onClick={handleClose} className={hmiBtnSecondary}>
            <X className={hmiIconBtn} />
          </button>
        </div>

        <p className={`mb-4 ${hmiAssistHint}`}>
          {active ? t('pf_holgura_omit_off_hint') : t('pf_holgura_omit_on_hint')}
        </p>

        <label className={hmiFieldLabel} htmlFor="pf-holgura-omit-employee">
          {t('maintenance_employee_label')}
        </label>
        <input
          id="pf-holgura-omit-employee"
          type="text"
          inputMode="numeric"
          maxLength={5}
          autoComplete="off"
          placeholder={t('maintenance_employee_placeholder')}
          value={employeeId}
          onChange={(e) => {
            setEmployeeId(e.target.value.replace(/\D/g, '').slice(0, 5));
            setError(null);
          }}
          className={`${hmiFieldInput} mb-3`}
        />

        {error === 'invalid_employee' && (
          <p className="mb-3 text-sm font-medium text-red-600 dark:text-red-400">
            {t('checklist_err_invalid_employee')}
          </p>
        )}
        {error === 'server' && (
          <p className="mb-3 text-sm font-medium text-red-600 dark:text-red-400">
            {t('pf_holgura_omit_error')}
          </p>
        )}

        <div className="flex flex-wrap gap-2 pt-1">
          <button
            type="button"
            disabled={submitting || employeeId.length !== 5}
            onClick={() => void handleConfirm()}
            className={hmiBtnPrimary}
          >
            {active ? t('pf_holgura_omit_confirm_off') : t('pf_holgura_omit_confirm_on')}
          </button>
          <button type="button" onClick={handleClose} className={hmiBtnSecondary}>
            {t('debug_mode_cancel')}
          </button>
        </div>
      </div>
    </div>
  );
};
