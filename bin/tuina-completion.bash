# Bash / Zsh programmable tab completion for TuinaDex v2.0 `tuina` CLI.
# Usage (Bash): source /path/to/TuinaDex_v2.0/bin/tuina-completion.bash
# Usage (Zsh):  autoload -U +X bashcompinit && bashcompinit && source /path/to/TuinaDex_v2.0/bin/tuina-completion.bash

_tuina_completions() {
    local cur prev word_idx cmd subcmd
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev=""
    if [ "${COMP_CWORD}" -gt 0 ]; then
        prev="${COMP_WORDS[COMP_CWORD-1]}"
    fi

    local global_opts="--help -h --version -v --dry-run --yes -y --env-info"
    local top_cmds="launch teleop collect calibrate arm hand data sys"

    # Find domain (word 1 ignoring leading global options)
    cmd=""
    subcmd=""
    for ((word_idx = 1; word_idx < COMP_CWORD; word_idx++)); do
        local w="${COMP_WORDS[word_idx]}"
        case "${w}" in
            --help|-h|--version|-v|--dry-run|--yes|-y|--env-info)
                continue
                ;;
            -*)
                continue
                ;;
            *)
                if [ -z "${cmd}" ]; then
                    cmd="${w}"
                elif [ -z "${subcmd}" ]; then
                    subcmd="${w}"
                fi
                ;;
        esac
    done

    if [ -z "${cmd}" ]; then
        if [[ "${cur}" == -* ]]; then
            COMPREPLY=($(compgen -W "${global_opts}" -- "${cur}"))
        else
            COMPREPLY=($(compgen -W "${top_cmds}" -- "${cur}"))
        fi
        return 0
    fi

    case "${cmd}" in
        launch)
            COMPREPLY=($(compgen -W "--target --sim --real --mock --mode --gui --no-gui --headless --use-scene-world --no-scene-world --physics-substeps --record --arm-channel --hand-port --rate-hz --watchdog-timeout ${global_opts}" -- "${cur}"))
            return 0
            ;;
        teleop)
            if [ -z "${subcmd}" ] && [[ "${cur}" != -* ]]; then
                COMPREPLY=($(compgen -W "hand arm composite real sim calibrate node" -- "${cur}"))
                return 0
            fi
            case "${subcmd}" in
                hand)
                    COMPREPLY=($(compgen -W "--backend --port --camera-id --rate --handedness --no-preview --mock --mock-camera --publish-ros2 --max-frames ${global_opts}" -- "${cur}"))
                    ;;
                arm)
                    COMPREPLY=($(compgen -W "--channel --camera-id --rate --mock --no-preview --max-frames ${global_opts}" -- "${cur}"))
                    ;;
                composite|real)
                    COMPREPLY=($(compgen -W "--arm-channel --hand-port --camera-id --rate-hz --mock --mock-hand --no-imu --no-gui --max-frames ${global_opts}" -- "${cur}"))
                    ;;
                sim)
                    COMPREPLY=($(compgen -W "--mock --no-gui --max-frames --solver-mode --axis-mapping --session-config --model ${global_opts}" -- "${cur}"))
                    ;;
                calibrate)
                    COMPREPLY=($(compgen -W "--imu-only --camera-id --imu-port --baudrate --mock --auto --no-gui --skip-table --skip-imu --skip-hand --output-session --output ${global_opts}" -- "${cur}"))
                    ;;
                node)
                    COMPREPLY=($(compgen -W "--rate-hz --camera-fps --mock --headless ${global_opts}" -- "${cur}"))
                    ;;
            esac
            return 0
            ;;
        collect)
            if [ -z "${subcmd}" ] && [[ "${cur}" != -* ]]; then
                COMPREPLY=($(compgen -W "hub terminal check" -- "${cur}"))
                return 0
            fi
            case "${subcmd}" in
                hub)
                    COMPREPLY=($(compgen -W "--env --mode --host --port --cameras --fps --ui-fps --task --repo-id --mock-teleop --standalone --no-standalone --camera-config --camera-map --max-staleness ${global_opts}" -- "${cur}"))
                    ;;
                terminal)
                    COMPREPLY=($(compgen -W "--env --mode --episodes --frames-per-episode --cameras --fps --task --repo-id --mock-teleop --standalone --no-standalone --headless --camera-config --camera-map --max-staleness ${global_opts}" -- "${cur}"))
                    ;;
                check)
                    COMPREPLY=($(compgen -W "--legacy ${global_opts}" -- "${cur}"))
                    ;;
            esac
            return 0
            ;;
        calibrate)
            if [ -z "${subcmd}" ] && [[ "${cur}" != -* ]]; then
                COMPREPLY=($(compgen -W "all preflight imu hand hand-limits arm-ready arm-goto" -- "${cur}"))
                return 0
            fi
            case "${subcmd}" in
                all|preflight)
                    COMPREPLY=($(compgen -W "--camera-id --imu-port --imu-axis-mapping --axis-mapping --skip-table --skip-imu --skip-hand --no-gui --mock --output ${global_opts}" -- "${cur}"))
                    ;;
                imu)
                    COMPREPLY=($(compgen -W "--port --baudrate --mock --auto --output-session --output-imu --output ${global_opts}" -- "${cur}"))
                    ;;
                hand)
                    COMPREPLY=($(compgen -W "--port --action --mock ${global_opts}" -- "${cur}"))
                    ;;
                hand-limits)
                    COMPREPLY=($(compgen -W "--port --out --motor --mock ${global_opts}" -- "${cur}"))
                    ;;
                arm-ready|arm-goto)
                    COMPREPLY=($(compgen -W "--channel ${global_opts}" -- "${cur}"))
                    ;;
            esac
            return 0
            ;;
        arm)
            if [ -z "${subcmd}" ] && [[ "${cur}" != -* ]]; then
                COMPREPLY=($(compgen -W "cli status watch drag read zero move teach goto-ready can" -- "${cur}"))
                return 0
            fi
            case "${subcmd}" in
                watch|drag|read)
                    COMPREPLY=($(compgen -W "--config -c --speed -s --channel --joint -j --hz ${global_opts}" -- "${cur}"))
                    ;;
                move)
                    COMPREPLY=($(compgen -W "--config -c --speed -s --channel --joint -j --angle -a --angles --wait --hold ${global_opts}" -- "${cur}"))
                    ;;
                can)
                    if [ "${prev}" = "can" ]; then
                        COMPREPLY=($(compgen -W "status up down vcan" -- "${cur}"))
                    else
                        COMPREPLY=($(compgen -W "--channel --bitrate ${global_opts}" -- "${cur}"))
                    fi
                    ;;
                *)
                    COMPREPLY=($(compgen -W "--config -c --speed -s --channel ${global_opts}" -- "${cur}"))
                    ;;
            esac
            return 0
            ;;
        hand)
            if [ -z "${subcmd}" ] && [[ "${cur}" != -* ]]; then
                COMPREPLY=($(compgen -W "control calibrate diagnose diagnostics teleop" -- "${cur}"))
                return 0
            fi
            case "${subcmd}" in
                control)
                    COMPREPLY=($(compgen -W "--port --mock ${global_opts}" -- "${cur}"))
                    ;;
                calibrate)
                    COMPREPLY=($(compgen -W "--port --action --mock ${global_opts}" -- "${cur}"))
                    ;;
                diagnose|diagnostics)
                    COMPREPLY=($(compgen -W "--port --out --motor --mock ${global_opts}" -- "${cur}"))
                    ;;
                teleop)
                    COMPREPLY=($(compgen -W "--port --camera-id --rate --filter-cutoff --filter-beta --handedness --no-preview --mock --mock-camera --publish-ros2 --max-frames ${global_opts}" -- "${cur}"))
                    ;;
            esac
            return 0
            ;;
        data)
            if [ -z "${subcmd}" ] && [[ "${cur}" != -* ]]; then
                COMPREPLY=($(compgen -W "convert convert-docker vla-cloud inspect" -- "${cur}"))
                return 0
            fi
            case "${subcmd}" in
                convert)
                    COMPREPLY=($(compgen -W "--bag --out --fps --allow-invalid ${global_opts}" -- "${cur}"))
                    ;;
                convert-docker|vla-cloud)
                    COMPREPLY=($(compgen -W "--build ${global_opts}" -- "${cur}"))
                    ;;
            esac
            return 0
            ;;
        sys)
            if [ -z "${subcmd}" ] && [[ "${cur}" != -* ]]; then
                COMPREPLY=($(compgen -W "env verify init-pc init-edge dds build test e2e rosbridge teleop-bridge pub probe docker completion" -- "${cur}"))
                return 0
            fi
            case "${subcmd}" in
                build)
                    COMPREPLY=($(compgen -W "--packages -p ${global_opts}" -- "${cur}"))
                    ;;
                test)
                    COMPREPLY=($(compgen -W "--suite ${global_opts}" -- "${cur}"))
                    ;;
                e2e)
                    COMPREPLY=($(compgen -W "--suite ${global_opts}" -- "${cur}"))
                    ;;
                rosbridge)
                    COMPREPLY=($(compgen -W "--port --address ${global_opts}" -- "${cur}"))
                    ;;
                pub)
                    COMPREPLY=($(compgen -W "--mode --position --arm-position --rate-hz --duration-sec ${global_opts}" -- "${cur}"))
                    ;;
                probe)
                    COMPREPLY=($(compgen -W "--position --rate-hz --active-sec --settle-sec ${global_opts}" -- "${cur}"))
                    ;;
                init-pc)
                    COMPREPLY=($(compgen -W "--install-rules --setup-vcan ${global_opts}" -- "${cur}"))
                    ;;
                init-edge)
                    COMPREPLY=($(compgen -W "--force --check-only ${global_opts}" -- "${cur}"))
                    ;;
                dds)
                    COMPREPLY=($(compgen -W "--install --generate-xml ${global_opts}" -- "${cur}"))
                    ;;
                completion)
                    COMPREPLY=($(compgen -W "--shell --install-hint ${global_opts}" -- "${cur}"))
                    ;;
            esac
            return 0
            ;;
    esac
}

complete -F _tuina_completions tuina
